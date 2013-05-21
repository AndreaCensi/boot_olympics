""" Functions for reading/writing ROS logs. """
from .. import np, contract, getLogger
import traceback
import os

logger = getLogger(__name__)

our_package = 'bootstrapping_adapter'
BootstrappingObservations_datatype = '%s/BootstrappingObservations' % our_package#@UnusedVariable


# Make sure we have our package in the path
from pkg_resources import resource_filename #@UnresolvedImport
root = resource_filename("bootstrapping_olympics", ".")
package_path = os.path.realpath(os.path.join(root, '..', '..', 'ros-packages'))
if not os.path.exists(package_path):
    logger.warning('Cannot find package path %r' % package_path)
else:
    current = os.environ.get('ROS_PACKAGE_PATH', '')
    if not package_path in current:
        #logger.debug('Adding %r to ROS_PACKAGE_PATH' % package_path)
        os.environ['ROS_PACKAGE_PATH'] = current + ':' + package_path 


try:
    import roslib 

    try:    
        roslib.load_manifest(our_package)
    except Exception as e:
        logger.error('Cannot load our package %s (%s)' % (our_package, e))
        current = os.environ.get('ROS_PACKAGE_PATH', '(unset)').split(":")
        logger.error('ROS_PACKAGE_PATH = %s' % current)
        raise 
    
    import ros
    import rospy
    import rosbag
    from bootstrapping_adapter.srv import BootstrappingCommands #@UnresolvedImport
    from bootstrapping_adapter.srv import BootstrappingCommandsResponse #@UnresolvedImport
    from bootstrapping_adapter.msg import BootstrappingObservations #@UnresolvedImport

    from ros import sensor_msgs, std_msgs #@UnresolvedImport
    from sensor_msgs.msg import Image as ROSImage
    from std_msgs.msg import (Float32MultiArray, MultiArrayDimension,
                              MultiArrayLayout)

    boot_has_ros = True #@UnusedVariable
    ros_error = None

except (ImportError, Exception) as e:
    boot_has_ros = False #@UnusedVariable
    ros_error = e

    logger.error('ROS support not available')
    logger.exception(e)
    logger.info('Continuing without ROS.') 
    #(traceback.format_exc())

else:
    from .ros_logs import *
    from .ros_script_utils import *
    from .launch_xml import *
    from .create_launch import *






