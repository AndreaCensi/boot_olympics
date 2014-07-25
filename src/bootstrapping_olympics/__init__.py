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
from .agent_states import *
from .logs import *
from .configuration import *
from .misc import *

# Try to load extra components (ROS, HDF, etc.)
from . import extra

from .programs.manager.batch.batch_learn import batch_jobs1
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from bootstrapping_olympics.programs.manager.batch.batch_manager import batch_process_manager

def jobs_comptests(context):
    
    from . import unittests
    warnings.warn('disabled')
    from comptests import jobs_registrar
    jobs_registrar(context, get_boot_config())

    from quickapp import iterate_context_names

    which = ["test_set1"]
    for c, id_set in iterate_context_names(context, which, key='set'):
        root = os.path.join(c.get_output_dir(), 'data_central')
        if not os.path.exists(root):
            os.makedirs(root)
            os.makedirs(os.path.join(root, 'config'))
        data_central = DataCentral(root)
        c.comp_config_dynamic(batch_process_manager, data_central, which_sets=id_set)
    
    
    




