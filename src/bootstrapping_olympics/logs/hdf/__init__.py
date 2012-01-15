''' Functions for dealing with HDF logs. '''

from .. import logger, np, contract

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

