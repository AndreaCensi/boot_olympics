#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy
import numpy as np
from pprint import pformat
from bootstrapping_adapter.srv import BootstrappingCommands
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec, check_valid_code_spec
from bootstrapping_olympics import AgentInterface
from ros_publisher import ROSPublisher
from bootstrapping_olympics.ros_scripts.ros_conversions import ROS2Python
import traceback, sys
from bootstrapping_olympics.ros_scripts.log_learn.learning_state import LearningStateDB
from bootstrapping_olympics.interfaces import BootSpec 


class Global:
    # Global variables -- a bit clumsy
    observations = None
    agent = None
    set_commands = None
    ros_publisher = None
    publish_interval = None
    publish = False
    count = 0
    ros2python = None
    
def handle_observations(resp):
    Global.observations = resp
   
def event_loop(observations):
    obs = Global.ros2python.convert(observations, filter_doubles=True) 
    if obs is not None: # possibly repeated
        Global.agent.process_observations(obs)

    commands = Global.agent.choose_commands()
    
    if Global.publish and Global.count % Global.publish_interval == 0:
        Global.agent.publish(Global.ros_publisher)
        Global.count += 1

    # TODO: check commands is compatible with commands_spec
    sender = '%s' % Global.agent.__class__.__name__
    Global.set_commands(commands=commands, sender=sender,
                timestamp=observations.timestamp)    

def agent_adapter(params, logger):   
    logger.info('My params:\n%s' % pformat(params))

    required = ['code', 'id_agent']
    optional = ['state_db_dir', 'publish_interval']
    for req in required:
        if not req in params:
            msg = 'Necessary parameter %r not given.' % req
            msg += '\nGiven configuration:\n%s' % pformat(params)
            raise Exception(msg)
        
    state_db_dir = params.get('state_db_dir', LearningStateDB.DEFAULT_DIR)
    id_agent = params['id_agent']
    code = params['code']
    
    Global.publish_interval = params.get('publish_interval', 0)
    Global.publish = Global.publish_interval > 0 
    Global.count = 0
    if Global.publish:
         Global.ros_publisher = ROSPublisher()
    
    check_valid_code_spec(code)
    
    AgentInterface.logger = RospyLogger(id_agent)
    
    Global.agent = instantiate_spec(code)   
    
    logger.info('agent instantiated: %s' % Global.agent)
    
    # Wait until the robot is connected
    result = rospy.wait_for_service('commands')
    logger.info('Connected to robot.')
    Global.set_commands = rospy.ServiceProxy('commands', BootstrappingCommands)
    # Subscribe to the observations
    subscriber = rospy.Subscriber('observations', BootstrappingObservations,
                     handle_observations)
    
    # Read one observation
    logger.info('Waiting for one observation...')
    while not rospy.is_shutdown():
        if Global.observations is not None:
            break
        rospy.sleep(0.1)
    # Close this subscriber
    subscriber.unregister()

    ob = Global.observations
    id_robot = ob.id_robot     
    spec = BootSpec.from_ros_structure(ob)
    Global.ros2python = ROS2Python(spec)

    logger.info('Obtained one observation; id_robot: %s, spec: %s' % (id_robot, spec))
    
    load_agent_state(Global.agent, id_agent, id_robot, state_db_dir)

    Global.agent.init(spec.sensels_shape, spec.commands_spec) # XXX
    
    # Subscribe to the observations
    subscriber = rospy.Subscriber('observations',
                                  BootstrappingObservations, event_loop)
    rospy.spin()

def load_agent_state(agent, id_agent, id_robot, state_db_directory):
    ''' Loads the agent state from the agent state DB in the given directory. '''
    db = LearningStateDB(state_db_directory)
    logger.info('Loading state for %s' % state_db_directory)
    key = dict(id_robot=id_robot, id_agent=id_agent)
    if db.has_state(**key):
        logger.info('Using previous learned state.')
        state = db.get_state(**key)
        logger.info('State after learning %d episodes.' % len(state.id_episodes))
        try:
            agent.set_state(state.agent_state)
        except:
            logger.error('Could not set agent to previous state.')
            raise 
    else:
        msg = ('No previous state for the agent/robot combination %r; I know %s.' 
               % (key, db.list_states()))
        logger.info(msg)

if __name__ == '__main__':
    class RospyLogger:
        def __init__(self, prefix):
            self.prefix = prefix
        def info(self, s):
            rospy.loginfo("%s:%s" % (self.prefix, s))
        def error(self, s):
            rospy.logerr("%s:%s" % (self.prefix, s))
            
    logger = RospyLogger('agent_adapter')
    rospy.init_node('agent_adapter')
    logger.info('agent_adapter started')    
    params = rospy.get_param('~')

    try:
        agent_adapter(params, logger)
        sys.exit(0)
    except rospy.ROSInterruptException: 
        sys.exit(0)
    except Exception as e:
        rospy.logerr('Bootstrapping adapter terminated due to an exception:\n%s'
                       % traceback.format_exc())
        sys.exit(-1)
        
