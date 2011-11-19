from .. import logger

try:
    import roslib #@UnresolvedImport
    roslib.load_manifest('bootstrapping_adapter')
    
    import rospy 

    from bootstrapping_adapter.srv import BootstrappingCommands #@UnresolvedImport
    from bootstrapping_adapter.srv import BootstrappingCommandsResponse #@UnresolvedImport
    from bootstrapping_adapter.msg import BootstrappingObservations #@UnresolvedImport

    from .ros_script_utils import *
    from .launch import *
    from .create_launch import *

except ImportError:
    logger.error('Could not load ROS library; ROS log support not available.')
    


  
