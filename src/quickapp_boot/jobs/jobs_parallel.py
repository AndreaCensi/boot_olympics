from quickapp_boot.utils import iterate_context_episodes
from quickapp_boot.programs import LearnLogNoSave
from contracts import contract
from bootstrapping_olympics.programs.manager import DataCentral
from quickapp import CompmakeContext
from compmake import Promise

__all__ = ['jobs_parallel_learning']

@contract(context=CompmakeContext, data_central=DataCentral,
          id_agent='str', id_robot='str', episodes='list(str)', returns=Promise)
def jobs_parallel_learning(context, data_central, id_agent, id_robot, episodes):
    """
        In this way, the agent learns separately on each log, and then
        the instances are merged using the merge() function.
    
        Needs:
        
            "episode-ready", id_robot, id_episode
            
        Returns the promise for the agent with learning complete. 
    """    
    agents = []
    for c, id_episode in iterate_context_episodes(context, episodes):
        c.needs('episode-ready', id_robot=id_robot, id_episode=id_episode)
        agent_i = c.subtask(LearnLogNoSave,
                            boot_root=data_central.get_boot_root(),
                            agent=id_agent,
                            robot=id_robot,
                            episodes=[id_episode],
                            add_job_prefix='')
        agents.append(agent_i)
        
    agent_state = jobs_merging_linear(context, agents)
    
    save = context.comp(save_state, data_central,
                        id_agent, id_robot, agent_state)
    
    return save

@contract(context=CompmakeContext, agents='list[>=2]', returns=Promise)
def jobs_merging_linear(context, agents):
    agent_state = agents[0]
    for i, a in enumerate(agents[1:]):
        agent_state = context.comp(merge_agents, agent_state, a,
                             job_id='merge-%d' % i) 
    return agent_state
    
def merge_agents(as1, as2):
    agent1, state1 = as1
    agent2, state2 = as2
    agent1.merge(agent2)
    state1.merge(state2)
    return agent1, state1


@contract(data_central=DataCentral, id_agent='str', id_robot='str')
def save_state(data_central, id_agent, id_robot, agent_state):
    agent, state = agent_state
#     state = LearningState(id_robot=id_robot, id_agent=id_agent)
    state.agent_state = agent.get_state()
#     state.id_episodes = set(id_episodes)
    db = data_central.get_agent_state_db() 
    db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
