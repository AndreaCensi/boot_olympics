from bootstrapping_olympics import LearningState, logger
from bootstrapping_olympics.utils import UserError, x_not_found
from contracts import contract
from bootstrapping_olympics.configuration.master import get_boot_config

__all__ = [
    'load_agent_state', 
    'load_agent_state_core',
]

@contract(returns='tuple(*,*)')
def load_agent_state(data_central, id_agent, id_robot,
                     reset_state=False,
                     raise_if_no_state=False):
    ''' 
        Load the agent, loading the agent state from the state_db directory.
        If the state is not available, then it initializes anew. 
        
        Returns a tuple (agent, state).
    '''
    logger.info('Loading state %s/%s reset=%s ' % (id_agent,
                                                   id_robot, reset_state))
    config = get_boot_config()
    agent = config.agents.instance(id_agent)  # @UndefinedVariable

    index = data_central.get_log_index()
    has_log = index.has_streams_for_robot(id_robot)
    has_instance = id_robot in config.robots 
    
    if has_log:
        boot_spec = index.get_robot_spec(id_robot)
    elif has_instance:
        robot = config.robots.instance(id_robot)
        boot_spec = robot.get_spec()
    else:
        msg = 'Cannot load agent state for %r ' % id_agent
        msg += 'because I cannot find any streams for robot %r ' % id_robot
        msg += 'and I need them to find the BootSpec. '
        msg += 'Also I could not instance the robot.'
        msg += x_not_found('robot', id_robot, index.list_robots())
        raise UserError(msg)
        
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
            state = db.reload_state_for_agent(id_agent=id_agent, id_robot=id_robot,
                                              agent=agent)
            return agent, state
        else:
            msg = 'No previous learned state found for %r/%r.' % (id_agent, id_robot)
            
            if raise_if_no_state:
                raise Exception(msg)
            
            logger.info(msg)
                
            state = LearningState(id_robot=id_robot, id_agent=id_agent)
            return agent, state

    assert False


