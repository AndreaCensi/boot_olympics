__version__ = '1.3'

import numpy as np
from contracts import contract

__docformat__ = 'restructuredtext'

# ## Setup logging
from logging import getLogger
from conf_tools.utils import col_logging  # colored logging

import logging
logging.basicConfig()

everybody_uses_same_logger = True
if everybody_uses_same_logger:
    logger = logging.getLogger('BO')
    getLogger = lambda _: logger
else:
    logger = logging.getLogger(__name__)

logger.setLevel(logging.DEBUG)


from . import utils

from .constants import *

# if False: # XXX
#    # This is just for documentation purposes
#    # assigns all symbols to bootstrapping_olympics
#    from .utils import assign_all_to_module
#    from . import interfaces
#    assign_all_to_module(interfaces)

from .interfaces import *
from .agent_states import *
from .logs import *
from .configuration import *


# Try to load extra components (ROS, HDF, etc.)
from . import extra

from .programs.manager.batch.batch_learn import batch_jobs1
