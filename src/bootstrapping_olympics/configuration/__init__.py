''' 
    Functions that deal with the configuration system. The heavy work
    is done by the ConfTools module. 
'''
from .. import logger

from .agents import *
from .robots import *
from .master import BootOlympicsConfig

from .yaml_ros_node_spec import *
