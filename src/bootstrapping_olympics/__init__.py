__version__ = '1.1'

import numpy as np
from contracts import contract

import logging
from conf_tools.utils import col_logging # colored logging

logger = logging.getLogger("BootOlympics")
logger.setLevel(logging.DEBUG)

from .constants import *


from .interfaces import *
from .configuration import BootOlympicsConfig


# Try to load ROS components
from .ros import *

if not boot_has_ros:
    logger.error('ROS support not available (%s).' % ros_error)

