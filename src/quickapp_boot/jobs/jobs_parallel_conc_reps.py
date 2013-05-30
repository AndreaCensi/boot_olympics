from .jobs_parallel import jobs_merging_linear, save_state
from bootstrapping_olympics.programs.manager import (DataCentral,
    get_agentstate_report)
from contracts import contract
from quickapp import CompmakeContext, iterate_context_names
from quickapp_boot.programs import LearnLogNoSaveHintRepeated


__all__ = ['jobs_parallel_learning_concurrent_reps']


@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', n='int,>=2',
          max_reps='int,>=2')
def jobs_parallel_learning_concurrent_reps(context, data_central, id_agent, id_robot, episodes,
                                      n, max_reps, intermediate_reports=True):
    """
        This one cycles multiple times in the data.
    """    
    agents = []
    
    contexts = ['learn%02dof%02d' % (i + 1, n) for i in range(n)]
        
    for i, (c, progress) in enumerate(iterate_context_names(context, contexts)):
        for id_episode in episodes: 
            c.needs('episode-ready', id_robot=id_robot, id_episode=id_episode)
         
        agent_i = c.subtask(LearnLogNoSaveHintRepeated,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent, robot=id_robot,
                            episodes=episodes,
                            parallel_hint=[i, n],
                            max_reps=max_reps,
                            add_job_prefix='')
        
        if intermediate_reports:
            report = context.comp(get_agentstate_report, agent_i, progress)
            context.add_report(report, 'agent_report_partial',
                               id_agent=id_agent, id_robot=id_robot,
                               progress=progress)
            
        agents.append(agent_i)


    agent_state = jobs_merging_linear(context, agents)
    
    save = context.comp(save_state, data_central,
                        id_agent, id_robot, agent_state)
    
    return save
