''' 
    Functions that deal with the configuration system. The heavy work
    is done by the ConfTools module. 
'''
from .. import getLogger

logger = getLogger(__name__)


from .master import BootOlympicsConfig, get_boot_config, set_boot_config

from .yaml_ros_node_spec import *

__all__ = ['BootOlympicsConfig', 'get_boot_config', 'set_boot_config']
