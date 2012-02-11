''' Functions for dealing with HDF logs. '''

from .. import np, contract, getLogger

logger = getLogger(__name__)


try:
    import tables

except ImportError:
    logger.info('Could not load PyTables library; '
                'HDF log support not available.')

else:
    from .utils import *
    from .index import *
    from .read import *
    from .write import *
    from .interface import *

