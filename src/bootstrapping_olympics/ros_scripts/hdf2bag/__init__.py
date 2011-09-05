# XXX: not sure about this 
import roslib; roslib.load_manifest('bootstrapping_adapter') #@UnresolvedImport

from ..log_learn.col_logging import * 

import logging
logger = logging.getLogger("hdf2bag")
logger.setLevel(logging.DEBUG)

from .conversion import *



