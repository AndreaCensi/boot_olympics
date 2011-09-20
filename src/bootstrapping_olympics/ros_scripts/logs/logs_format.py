from abc import abstractmethod
import os
import pickle

class LogsFormat:
    
    # extension -> LogsFormat
    formats = {}
    
    @abstractmethod
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
    
    def index_file_cached(self, filename):
        cache = '%s.index_cache' % filename
        if os.path.exists(cache): # TODO: mtime
            return pickle.load(open(cache))
        res = self.index_file(filename)
        with open(cache, 'wb') as f:
            pickle.dump(res, f, protocol=pickle.HIGHEST_PROTOCOL)
        return res

    @abstractmethod
    def read_stream(self, boot_stream):
        ''' Yields observations from the stream. '''

    @staticmethod
    def get_reader_for(filename):
        extension = os.path.splitext(filename)[1][1:]
        if not extension in LogsFormat.formats:
            msg = 'Cannot read %r, no reader for %r' % (filename, extension)
            raise ValueError(msg)
        return LogsFormat.formats[extension]
