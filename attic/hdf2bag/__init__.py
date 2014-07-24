import logging
logger = logging.getLogger("hdf2bag")
logger.setLevel(logging.DEBUG)

try: 
    from procgraph import pg
    from .conversion import *

except ImportError as e:
    import warnings
    msg = 'Skipping because of %s.' % e # XXX
    warnings.warn(msg)

