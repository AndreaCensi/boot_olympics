#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy, traceback
import time
from collections import namedtuple

from bootstrapping_adapter.srv import (BootstrappingCommands,
                                       BootstrappingCommandsResponse)
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec
import numpy as np

class Global:
    # Global variables -- a bit clumsy
    robot = None
    dt = 0.1
    publish = False
    
def commands_request(req):
    commands = np.array(req.commands)
    sender = req.sender
    Global.robot.apply_commands(commands=commands,
                                dt=Global.dt,
                                commands_source=sender)
    Global.publish = True
    return BootstrappingCommandsResponse(True)

def publish_observations(robot, publisher):
    obs = robot.get_observations()
    
    fields = {
        'timestamp': obs.timestamp,
        'sensel_values': obs.sensel_values,
        'sensel_shape': obs.sensel_values.shape,
        'commands': obs.commands,
        'commands_source': obs.commands_source,
        
        'counter': obs.counter,
        'id_episode': obs.id_episode,
        'id_environment': obs.id_environment,

        'id_robot': robot.id_robot,
        'id_actuators': robot.id_actuators,
        'id_sensors': robot.id_sensors,
        'commands_spec': robot.commands_spec.__repr__(),
        'type': 'sim'
    }
    msg = BootstrappingObservations(**fields)
    publisher.publish(msg)

def robot_adapter():   
    rospy.init_node('robot_adapter')
    
    params = rospy.get_param('~')
    rospy.loginfo('My params: %s' % params)


    Global.dt = params.get('dt', 0.1)
    sleep = params.get('sleep', 0.1)
    
    # TODO: check code is in the right format
    if not 'code' in params:
        raise Exception('No "code" to run specified.')
    code = params['code']
    
    rospy.loginfo('Using code = %r' % code)
    try:
        Global.robot = instantiate_spec(code)
    except Exception as e:
        msg = 'Could not instantiate robot code, using:\n'
        msg += ' class name: %s\n' % code[0]
        msg += ' parameters: %s\n' % code[1]
        msg += '\nThe following is the error given:\n' 
        msg += '%s' % traceback.format_exc()
        raise Exception(msg)
    
    publisher = rospy.Publisher('~observations', BootstrappingObservations, latch=True)
    
    # Reset simulation
    Global.robot.new_episode()
    
    # Start service
    service = rospy.Service('~commands',
                BootstrappingCommands, commands_request)

    # Broadcast observations anyway every few seconds
    last_observations_sent = 0
    maximum_interval = 2
    while not rospy.is_shutdown():
        now = time.time()
        publish = False
        if Global.publish:
            Global.publish = False
            publish = True
        if now > last_observations_sent + maximum_interval:
            last_observations_sent = now
            publish = True
        if publish: 
            publish_observations(Global.robot, publisher)
            
        rospy.sleep(sleep)

    
if __name__ == '__main__':
    try:
        robot_adapter()
    except rospy.ROSInterruptException: pass
