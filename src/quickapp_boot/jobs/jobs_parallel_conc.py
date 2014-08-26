
from boot_manager import DataCentral
from boot_manager.meat.publish_output import get_agentstate_report
from boot_manager.programs.manager.batch.main import save_state
from contracts import contract
from quickapp import CompmakeContext, iterate_context_names
from quickapp_boot import RM_EPISODE_READY
from quickapp_boot.programs import LearnLogNoSaveHint

__all__ = ['jobs_parallel_learning_concurrent', 'get_agentstate_report']


@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', n='int,>=2')
def jobs_parallel_learning_concurrent(context, data_central, id_agent, id_robot, episodes,
                                      n, intermediate_reports=True, final_report=True):
    """
        In this way, there are "n" instances of each agent. 
        Each instance sees all logs, but is given a hint using the 
        parallell_hint(i, n) function.
        The instances are merged using the merge() function.
    
        Needs:
        
            RM_EPISODE_READY, id_robot, id_episode 
    """    
    agents = []
    
    contexts = ['learn%02dof%02d' % (i + 1, n) for i in range(n)]
        
    for i, (c, progress) in enumerate(iterate_context_names(context, contexts)):
        for id_episode in episodes: 
            c.needs(RM_EPISODE_READY, id_robot=id_robot, id_episode=id_episode)
         
        agent_i = c.subtask(LearnLogNoSaveHint,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent, robot=id_robot,
                            episodes=episodes,
                            parallel_hint=[i, n],
                            add_job_prefix='')
        
        if intermediate_reports:
            report = c.comp(get_agentstate_report, agent_i, progress, job_id='report')
            c.add_report(report, 'agent_report_partial',
                               id_agent=id_agent, id_robot=id_robot,
                               progress=progress)
            
        agents.append(agent_i)


    agent_state = jobs_merging_recursive(context, agents)

    if final_report:
        report = context.comp(get_agentstate_report, agent_state, 'all', job_id='report')
        context.add_report(report, 'agent_report', id_agent=id_agent, id_robot=id_robot)

    save = context.comp(save_state, data_central,
                        id_agent, id_robot, agent_state)
    
    return save
 
    
