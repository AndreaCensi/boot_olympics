# XXX: not sure about this 
import roslib; roslib.load_manifest('bootstrapping_adapter') #@UnresolvedImport


import logging
logger = logging.getLogger("hdf2bag")
logger.setLevel(logging.DEBUG)

from .conversion import *



