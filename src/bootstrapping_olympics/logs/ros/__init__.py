''' Functions for dealing with ROS logs (currently broken). '''

from .. import LogsFormat, logger, np, contract

try:
    import roslib#@UnresolvedImport
    logger.debug('Loading ROS libraries...') 
    roslib.load_manifest('bootstrapping_adapter') 
    logger.debug('...done.')
except:
    logger.error('Could not load ROS library.')
    raise

from .index import *
from .ros_conversions import *
from .read import *
from .write import *
from .interface import *
