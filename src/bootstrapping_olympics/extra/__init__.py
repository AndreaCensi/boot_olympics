""" 
    These are a collections of modules that depend on external
    dependencies. If those are not installed, some functionality
    will not work, but we should be fine.
"""

# Logs in HDF
from . import hdf

# Do not load these automatically 
# from . import procgraph  # some video scripts assume already loaded 
    
if False:
    from . import ros
else:
    from bootstrapping_olympics import logger
    logger.warning('Temporarely removed ROS')



