""" Functions for reading/writing ROS logs. """
from .. import np, contract, getLogger


logger = getLogger(__name__)

BootstrappingObservations_datatype = \
        'bootstrapping_adapter/BootstrappingObservations'


try:
    import roslib #@UnresolvedImport
    roslib.load_manifest('bootstrapping_adapter')
    from ros import rospy
    from ros import rosbag
    from bootstrapping_adapter.srv import BootstrappingCommands
    from bootstrapping_adapter.srv import BootstrappingCommandsResponse
    from bootstrapping_adapter.msg import BootstrappingObservations

    from ros import sensor_msgs, std_msgs
    from sensor_msgs.msg import Image as ROSImage
    from std_msgs.msg import (Float32MultiArray, MultiArrayDimension,
                              MultiArrayLayout)

    boot_has_ros = True
    ros_error = None

except ImportError as e:
    boot_has_ros = False
    ros_error = e
    # export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ros-packages/
    logger.error('ROS support not available (%s).' % ros_error)

else:
    from .ros_logs import *
    from .ros_script_utils import *
    from .launch_xml import *
    from .create_launch import *






