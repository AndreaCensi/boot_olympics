#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter')
import rospy
import numpy as np
from bootstrapping_adapter.msg import BootstrappingObservations
from bootstrapping_olympics.loading import instantiate_spec

class Global:
    random_filter = None
    publisher = None

def event_loop(observations):
    observations.sensel_values = Global.random_filter.process(observations.sensel_values)
    Global.publisher.publish(observations)

def observation_filter_adapter():
    rospy.init_node('observations_filter_adapter')
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

    rospy.loginfo('Random filter instantiated: %s' % Global.random_filter)

    Global.publisher = rospy.Publisher('~observations', BootstrappingObservations, latch=True)
    
    subscriber = rospy.Subscriber('source',
                                  BootstrappingObservations,
                                  event_loop)
    rospy.spin()

if __name__ == '__main__':
    try:
        observation_filter_adapter()
    except rospy.ROSInterruptException: pass

