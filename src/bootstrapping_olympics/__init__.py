__version__ = '1.3'

import numpy as np
from contracts import contract
import os
import warnings
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

from streamels import StreamSpec, BootSpec, UnsupportedSpec

from .interfaces import *
from .configuration import *
from . import library

def jobs_comptests(context):
    # default is loaded 
    # from conf_tools import GlobalConfig
    # GlobalConfig.global_load_dirs(['bootstrapping_olympics.configs'])
    
    from . import unittests
    
    from comptests import jobs_registrar
    jobs_registrar(context, get_boot_config())




