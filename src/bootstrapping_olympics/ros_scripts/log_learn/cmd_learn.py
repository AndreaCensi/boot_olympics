from . import LearningState, LearningStateDB, bag_get_index_object, logger
from ...loading import BootOlympicsConfig, instantiate_spec
from optparse import OptionParser
from pprint import pformat
import numpy as np

usage = """
    learn  --agent <AGENT> --robot <ROBOT> [--reset]
    
"""
def cmd_learn_log(main_options, argv):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--reset", default=False, action='store_true',
                      help="Do not use cached state.")
    (options, args) = parser.parse_args(argv)
    
    if options.agent is None:
        msg = 'Please provide agent ID with --agent'
        logger.error(msg)
        return -1
    id_agent = options.agent

    if options.robot is None:
        msg = 'Please provide robot ID with --robot'
        logger.error(msg)
        return -1
    id_robot = options.robot
    
    index = bag_get_index_object(main_options.log_directory)
    robots = index['robots']
    
    if not id_robot in robots:
        msg = ('No log for robot %r found. I know: %s.' 
               % (id_robot, ", ".join(robots.keys())))
        logger.error(msg)
        return -3

    if not id_agent in BootOlympicsConfig.agents:
        msg = ('Agent %r not found in configuration. I know: %s.' 
               % (options.agent, ", ".join(BootOlympicsConfig.agents.keys())))
        logger.error(msg)
        return -3
    
    agent_spec = BootOlympicsConfig.agents[options.agent]
    
    logger.info('Instancing agent spec:\n%s' % pformat(agent_spec))
    
    agent = instantiate_spec(agent_spec['code'])   
    
    db = LearningStateDB(main_options.state_directory)
    key = dict(id_robot=id_robot, id_agent=id_agent)
    if not options.reset and db.has_state(**key):
        logger.info('Using previous learned state.')
        state = db.get_state(**key)
        logger.info('State after learning %d episodes.' % len(state.id_episodes))
        try:
            agent.set_state(state.agent_state)
        except:
            logger.error('Could not set agent to previous state.')
            raise
        
    else:
        state = LearningState(id_robot=id_robot, id_agent=id_agent,
                              id_episodes=set(), agent_state=None)
        
        # Finding shape
        for ob0 in robots[id_robot][0].read():
            break
        
        sensel_shape = tuple(ob0.sensel_shape) # XXX
        commands_spec = eval(ob0.commands_spec)
        logger.info('Agent init Sensels: %s  commands: %s' % 
                    (sensel_shape, commands_spec))
        agent.init(sensel_shape, commands_spec)

    # TODO: progress bar
            
    for stream in robots[id_robot]:
        # Check if all learned
        to_learn = stream.id_episodes.difference(state.id_episodes)
        if not to_learn:
            logger.info('Stream %s already completely learned.' % stream)
            continue
        else:
            logger.info('Learning %s' % to_learn)
        
        for observations in stream.read(only_episodes=to_learn):
            sensel_values = np.array(observations.sensel_values)
            agent.process_observations(sensel_values)
        
        state.id_episodes.update(to_learn)
        # Saving agent state
        state.agent_state = agent.get_state() 
        db.set_state(state=state, **key)
        
    
    
