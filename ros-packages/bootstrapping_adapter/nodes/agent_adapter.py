#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy

from bootstrapping_adapter.srv import BootstrappingCommands
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec

class Storage:
    observations = None

def handle_observations(resp):
    Storage.observations = resp
    rospy.loginfo('Received observations: %s' % resp)

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
        agent = instantiate_spec(code)
        # TODO: check right class
    except Exception as e:
        raise Exception('Could not instantiate agent: %s' % e)

    rospy.loginfo('agent instantiated: %s' % agent)
    
    # Wait until the robot is connected
    result = rospy.wait_for_service('commands')
    rospy.loginfo('Connected to robot.')
    set_commands = rospy.ServiceProxy('commands', BootstrappingCommands)
    # Subscribe to the observations
    rospy.Subscriber('observations', BootstrappingObservations, handle_observations)
    
    # Read one observation
    rospy.loginfo('Waiting for one observation')
    while not rospy.is_shutdown():
        if Storage.observations is not None:
            break
        rospy.sleep(0.1)
    
    rospy.loginfo('Obtained one observation')
    ob = Storage.observations
    rospy.loginfo('Here it is: %s' % ob)
    
    while not rospy.is_shutdown():
        str = "hello world %s" % rospy.get_time()
        rospy.loginfo(str)
        rospy.sleep(10.0)
    
if __name__ == '__main__':
    try:
        agent_adapter()
    except rospy.ROSInterruptException: pass
