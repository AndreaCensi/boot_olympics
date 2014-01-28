from contracts import contract

from bootstrapping_olympics.programs.manager import (DataCentral,
    get_agentstate_report)
from quickapp import CompmakeContext, iterate_context_names
from quickapp_boot import RM_EPISODE_READY
from quickapp_boot.jobs.jobs_parallel import jobs_merging_recursive
from quickapp_boot.programs import LearnLogNoSaveHintRepeated

from .jobs_parallel import save_state


__all__ = ['jobs_parallel_learning_concurrent_reps']


@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', n='int,>=2',
          max_reps='int,>=2')
def jobs_parallel_learning_concurrent_reps(context, data_central, id_agent, id_robot, episodes,
                                      n, max_reps, intermediate_reports=True,
                                      final_report=True):
    """
        This one cycles multiple times in the data.
    """    
    agents = []
    
    contexts = ['learn%02dof%02d' % (i + 1, n) for i in range(n)]
        
    for i, (c, progress) in enumerate(iterate_context_names(context, contexts)):
        extra_dep = []
        for id_episode in episodes:
            er = c.get_resource(RM_EPISODE_READY, id_robot=id_robot, id_episode=id_episode)
            extra_dep.append(er)
         
        agent_i = c.subtask(LearnLogNoSaveHintRepeated,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent, robot=id_robot,
                            episodes=episodes,
                            parallel_hint=[i, n],
                            max_reps=max_reps,
                            intermediate_reports=True,
                            add_job_prefix='',
                            extra_dep=extra_dep)
        
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
