from .. import logger, np, contract

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
    ros_error = 'everything is fine'

except ImportError as e:
    boot_has_ros = False
    ros_error = e

else:

    from .ros_logs import *
    from .ros_script_utils import *
    from .launch_xml import *
    from .create_launch import *





