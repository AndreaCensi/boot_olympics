''' Functions for dealing with HDF logs. '''

from .. import np, contract


import logging
logger = logging.getLogger("BO.logs.hdf")


try:
    import tables

except ImportError:
    logger.info('Could not load PyTables library; '
                'HDF log support not available.')

else:
    from .index import *
    from .read import *
    from .write import *
    from .interface import *

