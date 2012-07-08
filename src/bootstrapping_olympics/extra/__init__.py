""" 
    These are a collections of modules that depend on external
    dependencies. If those are not installed, some functionality
    will not work, but we should be fine.
"""

from .. import np, contract, getLogger

# Do not load these automatically 
from . import procgraph # some video scripts assume already loaded 
from . import ros
from . import hdf


