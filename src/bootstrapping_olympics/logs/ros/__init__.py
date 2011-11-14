''' Functions for dealing with ROS logs (currently broken). '''

from .. import LogsFormat, logger, np, contract

try:
    import roslib #@UnresolvedImport
    roslib.load_manifest('bootstrapping_adapter') 

    from .index import *
    from .ros_conversions import *
    from .read import *
    from .write import *
    from .interface import *


except ImportError:
    logger.info('Could not load ROS library; ROS log support not available.')
    
