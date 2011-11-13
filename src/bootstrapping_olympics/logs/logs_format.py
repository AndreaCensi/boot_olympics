from . import BootStream, logger
from abc import abstractmethod, ABCMeta
import os
import pickle

class LogsFormat:
    __metaclass__ = ABCMeta
    
    # extension -> LogsFormat
    formats = {}
    
    @abstractmethod
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
    
    # TODO: uniform mechanism for this
    def index_file_cached(self, filename, ignore_cache=False):
        cache = '%s.index_cache' % filename
        if os.path.exists(cache) and not ignore_cache: # TODO: mtime
            try:
                return pickle.load(open(cache))
            except Exception as e:
                msg = 'Could not unpickle cache %r, deleting.' % cache
                msg += '\n%s' % e
                os.unlink(cache) 
                logger.warning(msg)
                
        res = self.index_file(filename)
        for stream in res:
            assert isinstance(stream, BootStream)
        with open(cache, 'wb') as f:
            pickle.dump(res, f, protocol=pickle.HIGHEST_PROTOCOL)
        return res

    @abstractmethod
    def read_stream(self, boot_stream):
        ''' Yields observations from the stream. '''

    @abstractmethod
    def read_from_stream(self, filename, id_stream):
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

    
