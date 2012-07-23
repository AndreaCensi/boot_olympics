from contextlib import contextmanager
import time
from . import logger as my_logger

@contextmanager
def warn_long_time(max_wall_time, what, logger=None):
    if logger is None:
        logger = my_logger
        
    start = time.time()
    start_clock = time.clock()
    more_info = {}
    yield  more_info
    duration = time.time() - start
    duration_clock = time.clock() - start_clock
    if duration > max_wall_time:
        msg = ('Operation %s took %.3f wall time (clock: %.3f)' %
               (what, duration, duration_clock))
        if more_info:
            msg += '\nMore info: %s' % more_info
        logger.info(msg)


