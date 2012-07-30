""" 
    These are a collections of modules that depend on external
    dependencies. If those are not installed, some functionality
    will not work, but we should be fine.
"""

from .. import np, contract, getLogger, logger

# Do not load these automatically 
from . import procgraph # some video scripts assume already loaded 
    
if True:
    from . import ros
else:
    logger.warning('Temporarely removed ROS')

from . import hdf


