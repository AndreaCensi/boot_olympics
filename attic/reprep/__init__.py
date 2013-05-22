
try:
    import reprep
except ImportError as e:
    boot_has_reprep = False
    reprep_error = e
    from bootstrapping_olympics import logger
    logger.warning('RepRep support not available (%s).' % e)
else:
    boot_has_reprep = True
    reprep_error = None
    from .reprep_publisher import *


