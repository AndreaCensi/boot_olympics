from blocks import check_timed_named
from boot_manager import logger
from rawlogs_hdflog import HDFRawLog
import warnings

__all__ = [
    'hdf_read',
]


def hdf_read(filename, id_stream, boot_spec=None, read_extra=False,
              only_episodes=None):
    if only_episodes is not None:
        warnings.warn('only_episodes not implemented')
    r = HDFRawLog(filename)
    
    signals =  r.get_signals()
    # TODO: check pure
    topics = list(signals)
    topics.remove('boot_info')
    #print('reading stream %r episodes %s ' % (id_stream, only_episodes))
    #print('topics: %s' % topics)
    logger.error('Neeed to implement the only_episodes switch')
    for x in r.read(topics, start=None, stop=None):
        check_timed_named(x)
        #t, (s, v) = x
        #print('%.5f: %s' % (t, s))
        yield x



