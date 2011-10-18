from . import HDFLogWriter, hdf_list_streams, hdf_read
from .. import BootStream, LogsFormat
from contextlib import contextmanager

__all__ = ['HDFLogsFormat']

class HDFLogsFormat(LogsFormat): 
    
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        streams = hdf_list_streams(filename)
        for stream in streams:
            assert isinstance(stream, BootStream)
        return streams
    
    def read_stream(self, stream, read_extra=False):
        ''' Yields observations from the stream. '''
        for x in hdf_read(stream.bag_file,
                          stream.topic,
                          stream.spec, read_extra=read_extra):
            yield x

    
    @contextmanager
    def write_stream(self, filename, id_stream, boot_spec):
        writer = HDFLogWriter(filename, id_stream, boot_spec)
        try:
            yield writer
        finally:
            writer.close()

LogsFormat.formats['h5'] = HDFLogsFormat()
