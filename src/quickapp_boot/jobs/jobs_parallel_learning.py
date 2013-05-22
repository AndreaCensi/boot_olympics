from quickapp_boot.utils import iterate_context_episodes
from quickapp_boot.programs.learn_log_nosave import LearnLogNoSave
from bootstrapping_olympics.interfaces.agent import AgentInterface
from contracts import contract
from bootstrapping_olympics.agent_states.learning_state import LearningState
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral


def jobs_parallel_learning(context, data_central, id_agent, id_robot, episodes):
    """
    
        Needs:
        
            "episode-ready", id_robot, id_episode 
    """    
    agent = None
    i = 0
    for c, id_episode in iterate_context_episodes(context, episodes):
        i += 1
        c.needs('episode-ready', id_robot=id_robot, id_episode=id_episode)
         
        agent_i = c.subtask(LearnLogNoSave, boot_root=data_central.get_boot_root(),
                            agent=id_agent, robot=id_robot,
                            episodes=[id_episode],
                            add_job_prefix='')
        
        if agent is None:
            agent = agent_i
        else:
            agent = context.comp(merge_agents, agent, agent_i,
                                 job_id='merge-%d' % i) 
             
    save = context.comp(save_state, data_central, id_agent, id_robot, agent, episodes)
    
    return save


def merge_agents(agent1, agent2):
    print('merging')
    agent1.merge(agent2)
    return agent1


@contract(data_central=DataCentral, id_agent='str', id_robot='str', agent=AgentInterface,
          id_episodes='list(str)')
def save_state(data_central, id_agent, id_robot, agent, id_episodes):
    state = LearningState(id_robot=id_robot, id_agent=id_agent)
    state.agent_state = agent.get_state()
    state.id_episodes = set(id_episodes)
    db = data_central.get_agent_state_db() 
    db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
