#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy

from bootstrapping_adapter.srv import BootstrappingCommands
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec

class Global:
    # Global variables -- a bit clumsy
    observations = None
    agent = None
    set_commands = None
    
def handle_observations(resp):
    Global.observations = resp
    rospy.loginfo('Received observations: %s' % resp)

def event_loop(observations):
    sensel_values = observations.sensel_values
    Global.agent.process_observations(sensel_values)
    commands = Global.agent.choose_commands()
    sender = '%s' % Global.agent.__class__.__name__
    Global.set_commands(commands=commands, sender=sender,
                        timestamp=observations.timestamp)
    
def agent_adapter():   
    rospy.init_node('agent_adapter')
    rospy.loginfo('agent_adapter started')
    
    params = rospy.get_param('~')
    rospy.loginfo('My params: %s' % params)
    
    if not 'code' in params:
        raise Exception('No "code" to run specified.')
    code = params['code']
    
    rospy.loginfo('Using code = %r' % code)
    try:
        Global.agent = instantiate_spec(code)
        # TODO: check right class
    except Exception as e:
        raise Exception('Could not instantiate agent: %s' % e)

    rospy.loginfo('agent instantiated: %s' % Global.agent)
    
    # Wait until the robot is connected
    result = rospy.wait_for_service('commands')
    rospy.loginfo('Connected to robot.')
    Global.set_commands = rospy.ServiceProxy('commands', BootstrappingCommands)
    # Subscribe to the observations
    subscriber = rospy.Subscriber('observations', BootstrappingObservations,
                     handle_observations)
    
    # Read one observation
    rospy.loginfo('Waiting for one observation')
    while not rospy.is_shutdown():
        if Global.observations is not None:
            break
        rospy.sleep(0.1)
    # Close this subscriber
    subscriber.unregister()
    
    rospy.loginfo('Obtained one observation')
    ob = Global.observations
    sensel_shape = tuple(ob.sensel_shape) # XXX
    commands_spec = eval(ob.commands_spec)
    Global.agent.init(sensel_shape, commands_spec)
    
    rospy.loginfo('Here it is: %s' % ob)
    
    # Subscribe to the observations
    subscriber = rospy.Subscriber('observations',
                                  BootstrappingObservations,
                                  event_loop)
    rospy.spin()
    
if __name__ == '__main__':
    try:
        agent_adapter()
    except rospy.ROSInterruptException: pass
