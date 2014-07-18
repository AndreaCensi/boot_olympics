''' Functions for dealing with HDF logs. '''

try:
    import tables
except ImportError as e:
    boot_has_hdf = False
    hdf_error = e
    from bootstrapping_olympics import logger
    logger.warning('PyTables/HDF support not available (%s).' % e)
else:
    boot_has_hdf = True
    hdf_error = None
    from .utils import *
    from .index import *
    from .read import *
    from .write import *
    from .interface import *

