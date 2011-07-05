#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy
import time
from bootstrapping_adapter.srv import BootstrappingCommands
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec
from collections import namedtuple

class State:
    robot = None
    
def commands_request(req):
    rospy.loginfo('Received request: %s' % req)

def publish_observations(robot, publisher):
    obs = robot.get_observations()
    
    fields = {
        'timestamp': obs.timestamp,
        'sensel_values': obs.sensel_values,
        'commands': obs.commands,
        'commands_source': obs.commands_source,
        
        'counter': obs.counter,
        'id_episode': obs.id_episode,
        'id_environment': obs.id_environment,

        'id_robot': robot.id_robot,
        'id_actuators': robot.id_actuators,
        'id_sensors': robot.id_sensors,
        'type': 'sim'
    }
    msg = BootstrappingObservations(**fields)
    publisher.publish(msg)

def robot_adapter():   
    rospy.init_node('robot_adapter')
    
    params = rospy.get_param('~')
    rospy.loginfo('My params: %s' % params)
    
    if not 'code' in params:
        raise Exception('No "code" to run specified.')
    code = params['code']
    
    rospy.loginfo('Using code = %r' % code)
    try:
        State.robot = instantiate_spec(code)
    except Exception as e:
        raise Exception('Could not instantiate agent: %s' % e)
    
    publisher = rospy.Publisher('~observations', BootstrappingObservations)
    
    # Reset simulation
    State.robot.next_episode()
    
    # Start service
    service = rospy.Service('~commands',
                BootstrappingCommands, commands_request)

    # Broadcast observations anyway every few seconds
    last_observations_sent = 0
    maximum_interval = 2
    while not rospy.is_shutdown():
        now = time.time()
        if now > last_observations_sent + maximum_interval: 
            publish_observations(State.robot, publisher)
            last_observations_sent = now
        rospy.sleep(0.01)

    
if __name__ == '__main__':
    try:
        robot_adapter()
    except rospy.ROSInterruptException: pass
