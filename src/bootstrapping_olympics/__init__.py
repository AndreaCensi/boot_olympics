__version__ = '1.1'

import numpy as np
from contracts import contract

from logging import getLogger
from conf_tools.utils import col_logging # colored logging

import logging
logging.basicConfig()
logger = logging.getLogger("BO")

logger.setLevel(logging.DEBUG)
# XXX: let everybody use the same logger
getLogger = lambda name: logger

from .constants import *


from .interfaces import *
from .agent_states import *
from .logs import *
from .configuration import BootOlympicsConfig


# Try to load ROS components
from .ros import *

from .programs.manager.batch.batch_learn import batch_jobs1

