from . import BootStream, logger
from abc import abstractmethod, ABCMeta
import os
from bootstrapping_olympics.utils.safe_pickle import safe_pickle_load, \
    safe_pickle_dump
from bootstrapping_olympics.utils.warn_long_time_exc import warn_long_time
from conf_tools.utils.friendly_paths import friendly_path


class LogsFormat:
    __metaclass__ = ABCMeta

    # extension -> LogsFormat
    formats = {}

    @abstractmethod
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''

    # TODO: uniform mechanism for this
    # TODO: this is not really concurrent friendly
    def index_file_cached(self, filename, ignore_cache=False):
        cache = '%s.index_cache' % filename
        if os.path.exists(cache) and not ignore_cache: # TODO: mtime
            try:
                return safe_pickle_load(cache)
            except Exception as e:
                msg = 'Could not unpickle cache %r, deleting.' % friendly_path(cache)
                msg += '\n%s' % e
                logger.warning(msg)
                try:
                    os.unlink(cache)
                except:
                    pass
        logger.debug('Indexing file %r' % friendly_path(filename))
        res = self.index_file(filename)
        for stream in res:
            assert isinstance(stream, BootStream)
            
        logger.debug('Now dumping file %r' % friendly_path(cache))
        with warn_long_time(1, 'dumping %r' % friendly_path(cache)):
            safe_pickle_dump(res, cache, protocol=2)

        return res

    @abstractmethod
    def read_stream(self, boot_stream, only_episodes=None):
        ''' Yields observations from the stream. '''

    @abstractmethod
    def read_from_stream(self, filename, id_stream, only_episodes=None):
        ''' Yields observations from the stream. '''

    @abstractmethod
    def write_stream(self, filename, id_stream, boot_spec):
        ''' Yields a writer object (interface TBD). '''

    @staticmethod
    def get_reader_for(filename):
        extension = os.path.splitext(filename)[1][1:]
        if not extension in LogsFormat.formats:
            msg = 'Cannot read %r, no reader for %r.' % (filename, extension)
            raise ValueError(msg)
        return LogsFormat.formats[extension]


