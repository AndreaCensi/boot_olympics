import logging;  logging.basicConfig()
from conf_tools.utils import col_logging # colored logging

logger = logging.getLogger("BootOlympics")
logger.setLevel(logging.DEBUG)


import numpy as np
from contracts import contract

from .interfaces import *
from .configuration import BootOlympicsConfig
