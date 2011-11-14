from bootstrapping_olympics.agent_states.learning_state import LearningState

from . import logger

def load_agent_state(data_central, id_agent, id_robot,
                     reset_state=False,
                     raise_if_no_state=False):
    ''' 
        Load the agent, loading the agent state from the state_db directory.
        If the state is not available, then it initializes anew. 
        
        Returns a tuple (agent, state).
    '''
    logger.info('Loading state %s/%s reset=%s ' % (id_agent, id_robot, reset_state))
    agent = data_central.get_bo_config().agents.instance(id_agent) #@UndefinedVariable

    index = data_central.get_log_index()
    boot_spec = index.get_robot_spec(id_robot)
    agent.init(boot_spec)

    return load_agent_state_core(data_central, id_agent, agent, id_robot,
                          reset_state=reset_state,
                          raise_if_no_state=raise_if_no_state)
    
def load_agent_state_core(data_central, id_agent, agent, id_robot,
                          reset_state=False,
                          raise_if_no_state=False):

    db = data_central.get_agent_state_db()
    key = dict(id_robot=id_robot, id_agent=id_agent)

    if reset_state:
        state = LearningState(id_robot=id_robot, id_agent=id_agent)
        return agent, state
    else:
        if db.has_state(**key):
            logger.info('Using previous learned state.')
            state = db.reload_state_for_agent(id_agent=id_agent,
                                              id_robot=id_robot,
                                              agent=agent)
            return agent, state
        else:
            logger.info('No previous learned state found.')
            if raise_if_no_state:
                raise Exception('No previous learned state found.')
            else:
                state = LearningState(id_robot=id_robot, id_agent=id_agent)
                return agent, state
    
    assert False
    
    
