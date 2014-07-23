from rawlogs_hdflog import HDFRawLog
import warnings


__all__ = [
    'hdf_read',
]


def hdf_read(filename, id_stream, boot_spec=None, read_extra=False,
              only_episodes=None):
    
    r = HDFRawLog(filename)
    
    signals =  r.get_signals()
    # TODO: check pure
    topics = list(signals)
    topics.remove('boot_info')
    warnings.warn('Neeed to implement the only_episodes switch')
    for x in r.read(topics, start=None, stop=None):
        yield x




