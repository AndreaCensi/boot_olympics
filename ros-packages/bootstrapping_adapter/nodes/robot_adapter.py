#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy, traceback
import time
from pprint import pformat
from collections import namedtuple

from bootstrapping_adapter.srv import (BootstrappingCommands,
                                       BootstrappingCommandsResponse)
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec, check_valid_code_spec
import sys
import numpy as np

class Global:
    # Global variables -- a bit clumsy
    robot = None 
    publish = False
    
def commands_request(req):
    commands = np.array(req.commands)
    sender = req.sender
    Global.robot.set_commands_wrap(commands=commands,
                                commands_source=sender)
    Global.publish = True
    return BootstrappingCommandsResponse(True)

def publish_observations(robot, publisher):
    obs = robot.get_observations_wrap()
    # XXX: this will give empty commands at the beginning
    
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
    
    rospy.loginfo('My params:\n%s' % pformat(params))

    required = ['code']
    for req in required:
        if not req in params:
            msg = 'Necessary parameter %r not given.' % req
            msg += '\nGiven configuration:\n%s' % pformat(params)
            raise Exception(msg)


    Global.dt = params.get('dt', 0.1)
    sleep = params.get('sleep', 0.0)
    
    code = params['code']
    rospy.loginfo('Using code: %s' % pformat(code))
    check_valid_code_spec(code)
    try:
        Global.robot = instantiate_spec(code)
    except:
        msg = 'Could not instantiate robot code, using:\n'
        msg += ' class name: %s\n' % code[0]
        msg += ' parameters: %s\n' % code[1]
        rospy.logerr(msg)
        raise
    
    publisher = rospy.Publisher('~observations',
                                BootstrappingObservations, latch=True)
    
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
        sys.exit(0)
    except rospy.ROSInterruptException: 
        sys.exit(0)
    except Exception as e:
        rospy.logerr('Robot adapter terminated due to an exception:\n%s'
                       % traceback.format_exc())
        sys.exit(-1)

