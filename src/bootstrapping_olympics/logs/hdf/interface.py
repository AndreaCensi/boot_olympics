from . import HDFLogWriter, hdf_list_streams, hdf_read
from .. import LogsFormat
from contextlib import contextmanager

__all__ = ['HDFLogsFormat']


class HDFLogsFormat(LogsFormat):

    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        return hdf_list_streams(filename)

    def read_stream(self, stream, read_extra=False):
        ''' Yields observations from the stream. '''
        for x in hdf_read(filename=stream.get_filename(),
                          id_stream=stream.get_topic(),
                          boot_spec=stream.get_spec(), read_extra=read_extra):
            yield x

    def read_from_stream(self, filename, id_stream, read_extra=False):
        ''' Yields observations from the stream. '''
        for x in hdf_read(filename=filename,
                          id_stream=id_stream,
                          boot_spec=None,
                          read_extra=read_extra):
            yield x

    @contextmanager
    def write_stream(self, filename, id_stream, boot_spec):
        writer = HDFLogWriter(filename, id_stream, boot_spec)
        try:
            yield writer
        finally:
            writer.close()

LogsFormat.formats['h5'] = HDFLogsFormat()
