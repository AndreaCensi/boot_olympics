#!/usr/bin/env python
import roslib; roslib.load_manifest('bootstrapping_adapter') #@UnresolvedImport
import rospy #@UnresolvedImport
from bootstrapping_olympics.ros.adapters.boot_robot_adapter import boot_robot_adapter_main
from bootstrapping_olympics.ros.ros_script_utils.wrap import wrap_ros_script

def main():
    rospy.init_node('robot_adapter')
    params = rospy.get_param('~')
    boot_robot_adapter_main(params)

if __name__ == '__main__':
    wrap_ros_script(main)    
