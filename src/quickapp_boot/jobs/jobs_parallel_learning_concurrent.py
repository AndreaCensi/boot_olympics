from .jobs_parallel_learning import jobs_merging_linear, save_state
from bootstrapping_olympics.programs.manager import DataCentral
from contracts import contract
from quickapp import CompmakeContext, iterate_context_names
from quickapp_boot.programs.learn_log_nosave_hint import LearnLogNoSaveHint
from bootstrapping_olympics.programs.manager.meat.publish_output import get_agent_report_from_state

__all__ = ['jobs_parallel_learning_concurrent']


@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', n='int,>=2')
def jobs_parallel_learning_concurrent(context, data_central, id_agent, id_robot, episodes,
                                      n, intermediate_reports=True):
    """
        In this way, there are "n" instances of each agent. 
        Each instance sees all logs, but is given a hint using the 
        parallell_hint(i, n) function.
        The instances are merged using the merge() function.
    
        Needs:
        
            "episode-ready", id_robot, id_episode 
    """    
    agents = []
    
    contexts = ['learn%02dof%02d' % (i + 1, n) for i in range(n)]
        
    for i, (c, progress) in enumerate(iterate_context_names(context, contexts)):
        for id_episode in episodes: 
            c.needs('episode-ready', id_robot=id_robot, id_episode=id_episode)
         
        agent_i = c.subtask(LearnLogNoSaveHint,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent, robot=id_robot,
                            episodes=episodes,
                            parallel_hint=[i, n],
                            add_job_prefix='')
        
        if intermediate_reports:
            report = context.comp(get_agent_report, agent_i, progress)
            context.add_report(report, 'agent_report_partial',
                               id_agent=id_agent, id_robot=id_robot,
                               progress=progress)
            
        agents.append(agent_i)


    agent_state = jobs_merging_linear(context, agents)
    
    save = context.comp(save_state, data_central,
                        id_agent, id_robot, agent_state)
    
    return save

@contract(agent_state='tuple', progress='str')
def get_agent_report(agent_state, progress):
    agent, state = agent_state
    return get_agent_report_from_state(agent, state, progress)  
    
    
    
