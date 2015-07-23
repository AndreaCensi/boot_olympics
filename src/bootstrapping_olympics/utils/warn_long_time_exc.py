from . import logger as my_logger
from StringIO import StringIO
from conf_tools.utils import friendly_path
from contextlib import contextmanager
import os
import time

@contextmanager
def warn_long_time(max_wall_time, what, logger=None):
    # XXX: 201304: I don't remember why I was using the deprecation
    # my_logger.warning('Deprecation warning for warn_long_time (%s)' % what)
    if logger is None:
        logger = my_logger    
    start = time.time()
    start_clock = time.clock()
    more_info = {}
    
    yield more_info
    
    duration = time.time() - start
    duration_clock = time.clock() - start_clock
    if duration > max_wall_time:
        msg = ('Operation took %.5f wall time (clock: %.5f) %s' %
               (duration, duration_clock, what))
        if more_info:
            msg += '\nMore info: %s' % more_info
        logger.info(msg)
        


@contextmanager
def warn_long_time2(max_wall_time, what=None, logger=None):
    if logger is None:
        logger = my_logger    
    start = time.time()
    start_clock = time.clock()
    more_info = StringIO()
    
    yield more_info
    
    duration = time.time() - start
    duration_clock = time.clock() - start_clock
    if duration > max_wall_time:
        if what is None:
            what = ""
        msg = ('Operation %s took %.3f wall time (clock: %.3f)' % 
               (what, duration, duration_clock))
        msg += '\nMore info: %s' % more_info.getvalue()
        logger.info(msg)


default_wait = 1.0  # seconds

@contextmanager
def warn_long_time_writing(filename, max_wall_time=default_wait, logger=None):
    """ Warns if it takes a long time to do whatever
        you do in the context. """
    # TODO: use timer for logging
    with warn_long_time2(max_wall_time, logger=logger) as more_info:
        yield
        more_info.write('Written file %s of size %s' % 
                        (friendly_path(filename), friendly_filesize(filename)))


@contextmanager
def warn_long_time_reading(filename, max_wall_time=default_wait, logger=None):
    """ Warns if it takes a long time to do whatever
        you do in the context. """
    # TODO: use timer for logging
    with warn_long_time2(max_wall_time, logger=logger) as more_info:
        more_info.write('Reading file %s of size %s' % 
                        (friendly_path(filename), friendly_filesize(filename)))
        yield


def friendly_filesize(filename):
    size = os.stat(filename).st_size
    if size > 1e6:
        return '%.1fMB' % (size / 1e6)
    if size > 1e3:
        return '%.1fKB' % (size / 1e3)
    return  '%.1fb' % (size)
        
        
        
    
