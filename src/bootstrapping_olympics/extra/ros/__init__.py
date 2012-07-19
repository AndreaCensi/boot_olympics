""" Functions for reading/writing ROS logs. """
from .. import np, contract, getLogger
import traceback

logger = getLogger(__name__)


BootstrappingObservations_datatype = 'bootstrapping_adapter/BootstrappingObservations' #@UnusedVariable

try:
    import roslib #@UnresolvedImport
    try:
        roslib.load_manifest('bootstrapping_adapter')
    except Exception as e:
        logger.error('Cannot load our package bootstrapping_adapter (%s)' % e)
        logger.error('Probably you need: \n'
                     'export ROS_PACKAGE_PATH='
                     '${B11_SRC}/bootstrapping_olympics/ros-packages:${ROS_PACKAGE_PATH}')
        raise 
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

except (ImportError, Exception) as e:
    boot_has_ros = False
    ros_error = e
    # export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ros-packages/
    logger.error('ROS support not available')
    logger.error(traceback.format_exc())

else:
    from .ros_logs import *
    from .ros_script_utils import *
    from .launch_xml import *
    from .create_launch import *






