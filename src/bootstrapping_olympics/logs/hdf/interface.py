from . import hdf_list_streams, hdf_read
from .. import LogsFormat
from.. import BootStream

class HDFLogsFormat(LogsFormat): 
    
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        streams = hdf_list_streams(filename)
        for stream in streams:
            assert isinstance(stream, BootStream)
        print streams
        return streams
    
    def read_stream(self, stream):
        ''' Yields observations from the stream. '''
        for x in hdf_read(stream.bag_file,
                          stream.topic,
                          stream.spec):
            yield x


LogsFormat.formats['h5'] = HDFLogsFormat()
