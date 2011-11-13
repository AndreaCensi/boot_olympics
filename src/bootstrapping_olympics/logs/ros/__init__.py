''' Functions for dealing with ROS logs (currently broken). '''

from .. import LogsFormat, logger, np, contract

try:
    import roslib#@UnresolvedImport
    logger.debug('Loading ROS libraries...') 
    roslib.load_manifest('bootstrapping_adapter') 
    logger.debug('...done.')


    from .index import *
    from .ros_conversions import *
    from .read import *
    from .write import *
    from .interface import *


except ImportError:
    logger.info('Could not load ROS library; ROS log support not available.')
    
