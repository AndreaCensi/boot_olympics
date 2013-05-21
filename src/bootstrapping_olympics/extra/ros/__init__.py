import traceback
import os
from bootstrapping_olympics import logger

try:
    import roslib
    roslib.load_manifest('sensor_msgs')
    import ros
    import rospy
    import rosbag
     
    from sensor_msgs.msg import Image as ROSImage
    from std_msgs.msg import (Float32MultiArray, MultiArrayDimension,
                              MultiArrayLayout)

    boot_has_ros = True  # @UnusedVariable
    ros_error = None

except (ImportError, Exception) as e:
    boot_has_ros = False  # @UnusedVariable
    ros_error = e

    logger.error('ROS support not available')
    logger.exception(e)
    logger.info('Continuing without ROS.') 
    # (traceback.format_exc())

else:
    from .ros_script_utils import *
    from .publisher import *
    





