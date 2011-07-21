#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy
import numpy as np
from bootstrapping_olympics.loading import instantiate_spec
from bootstrapping_adapter.srv import (BootstrappingCommands,
                                       BootstrappingCommandsResponse)
class Global:
    random_filter = None
    set_commands = None

def filter_commands(commands):
    commands.commands = Global.random_filter.process(commands.commands)
    Global.set_commands(commands)
    return BootstrappingCommandsResponse(True)

def observation_filter_adapter():
    rospy.init_node('commands_filter_adapter')
    rospy.loginfo('observations_filter_adapter started')
    
    params = rospy.get_param('~')
    rospy.loginfo('My params: %s' % params)
    
    if not 'code' in params:
        raise Exception('No "code" to run specified.')
    code = params['code']
        
    rospy.loginfo('Using code = %r' % code)
    try:
        Global.random_filter = instantiate_spec(code)        
        # TODO: check right class
    except Exception as e:
        raise Exception('Could not instantiate agent: %s' % e)

    rospy.loginfo('Commands filter instantiated: %s' % Global.random_filter)

    Global.set_commands = rospy.ServiceProxy('commands', BootstrappingCommands)

    service = rospy.Service('~commands',
                BootstrappingCommands, filter_commands)
    rospy.spin()

if __name__ == '__main__':
    try:
        observation_filter_adapter()
    except rospy.ROSInterruptException: pass

