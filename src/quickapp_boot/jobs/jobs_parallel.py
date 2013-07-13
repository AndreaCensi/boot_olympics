from bootstrapping_olympics.programs.manager import (DataCentral,
    get_agentstate_report)
from compmake import Promise
from contracts import contract
from quickapp import CompmakeContext, iterate_context_names
from quickapp_boot import RM_EPISODE_READY
from quickapp_boot.programs import LearnLogNoSave
import numpy as np

__all__ = ['jobs_parallel_learning']

@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', returns=Promise)
def jobs_parallel_learning(context, data_central, id_agent, id_robot, episodes,
                           intermediate_reports=True, final_report=True,
                           episodes_per_tranche=1):
    """
        In this way, the agent learns separately on each log, and then
        the instances are merged using the merge() function.
    
        Needs:
        
            "episode-ready", id_robot, id_episode
            
        Returns the promise for the agent with learning complete
        (tuple agent, state) 
    """    
    agents = []
    
    tranches = get_tranches(episodes, episodes_per_tranche=episodes_per_tranche)
    ntranches = len(tranches)
    tranches_names = ['tranche%02d' % i for i in range(ntranches)]
    
    for i, (c, _) in enumerate(iterate_context_names(context, tranches_names)):
        tranche_episodes = tranches[i]
        extra_dep = []
        for id_episode in tranche_episodes:
            er = c.get_resource(RM_EPISODE_READY, id_robot=id_robot, id_episode=id_episode)
            extra_dep.append(er)
        
        agent_i = c.subtask(LearnLogNoSave,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent,
                            robot=id_robot,
                            episodes=tranche_episodes,
                            add_job_prefix='',
                            extra_dep=extra_dep)
        agents.append(agent_i)
     
        if intermediate_reports and ntranches > 1:
            progress = '%s' % id_episode
            report = c.comp_config(get_agentstate_report, agent_i, progress, job_id='report')
            c.add_report(report, 'agent_report_partial',
                               id_agent=id_agent, id_robot=id_robot,
                               progress=progress)
         
    agent_state = jobs_merging_recursive(context, agents)
    
    save = context.comp_config(save_state, data_central,
                        id_agent, id_robot, agent_state)
    
    if final_report:
        report = context.comp(get_agentstate_report, agent_state, 'all', job_id='report')
        context.add_report(report, 'agent_report', id_agent=id_agent, id_robot=id_robot)
         
    return save

# @contract(context=CompmakeContext, agents='list[>=1]', returns=Promise)
# def jobs_merging_linear(context, agents):
#     """ merges ((A1, A2), A3), A4), ...) """ 
#     agent_state = agents[0]
#     for i, a in enumerate(agents[1:]):
#         agent_state = context.comp(merge_agents, agent_state, a,
#                              job_id='merge-%d' % i) 
#     return agent_state

@contract(context=CompmakeContext, agents='list[>=1]', returns=Promise)
def jobs_merging_recursive(context, agents):
    """ merges hyerarchically """
    n = len(agents)
    if n == 1:
        return agents[0]
    
    half = n / 2
    a1 = agents[0:half]
    a2 = agents[half:]
    assert len(a1) > 0
    assert len(a2) > 0
    
    m1 = jobs_merging_recursive(context, a1)
    m2 = jobs_merging_recursive(context, a2)
    return context.comp(merge_agents, m1, m2)
      
    
def merge_agents(as1, as2):
    agent1, state1 = as1
    agent2, state2 = as2
    agent1.merge(agent2)
    state1.merge(state2)
    return agent1, state1


@contract(data_central=DataCentral, id_agent='str', id_robot='str')
def save_state(data_central, id_agent, id_robot, agent_state):
    agent, state = agent_state
    state.agent_state = agent.get_state()
    db = data_central.get_agent_state_db() 
    db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
    return agent_state

@contract(returns='list(list(str))')
def get_tranches(ids, episodes_per_tranche=10):
    """ Returns a list of list """
    l = []
    num_tranches = int(np.ceil(len(ids) * 1.0 / episodes_per_tranche))
    for t in range(num_tranches):
        e_from = t * episodes_per_tranche
        e_to = min(len(ids), e_from + episodes_per_tranche)
        l.append([ids[i] for i in range(e_from, e_to)])
    return l
