from .. import logger

try:
    import roslib #@UnresolvedImport
    roslib.load_manifest('bootstrapping_adapter')
    
    import rospy 

    from .create_launch import *
    
    logger.info('ROS components loaded.')
    
    from bootstrapping_adapter.srv import BootstrappingCommands #@UnresolvedImport
    from bootstrapping_adapter.srv import BootstrappingCommandsResponse #@UnresolvedImport
    from bootstrapping_adapter.msg import BootstrappingObservations #@UnresolvedImport


except ImportError:
    logger.info('Could not load ROS library; ROS log support not available.')
    


  
