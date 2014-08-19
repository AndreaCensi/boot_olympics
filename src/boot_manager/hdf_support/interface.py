from .index import hdf_list_streams
from .read import hdf_read
from .write import HDFLogWriter2
from boot_manager import LogsFormat, logger
from bootstrapping_olympics.utils import safe_write_tmp_filename
from contextlib import contextmanager
from contracts import raise_wrapped

__all__ = ['HDFLogsFormat2']


class HDFLogsFormat2(LogsFormat):

    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        try:
            return hdf_list_streams(filename)
        except BaseException as e:
            raise_wrapped(ValueError, e, 'Error while indexing file.',
                          filename=filename)

    def read_stream(self, stream, read_extra=False, only_episodes=None,
                    check_valid_values=False):
        ''' Yields timestamp, (signal, value). '''
        spec = stream.get_spec()
        for x in hdf_read(filename=stream.get_filename(),
                          id_stream=stream.get_topic(),
                          boot_spec=spec,
                          read_extra=read_extra,
                          only_episodes=only_episodes):
            if check_valid_values:
                spec.get_commands().check_valid_value(x['commands'])
                spec.get_observations().check_valid_value(x['observations'])
                
            yield x

    def read_from_stream(self, filename, id_stream, read_extra=False,
                         only_episodes=None):
        ''' Yields observations from the stream. '''
        for x in hdf_read(filename=filename,
                          id_stream=id_stream,
                          boot_spec=None,
                          only_episodes=only_episodes,
                          read_extra=read_extra):
            yield x

    @contextmanager
    def write_stream(self, filename, id_stream, boot_spec, id_agent, id_robot):
        writer = HDFLogWriter2(filename, id_stream, boot_spec, id_agent, id_robot)
        try:
            yield writer
        except:
            logger.error('Got exception in the write_stream() context manager')
            writer.cleanup()
            raise
        else:
            writer.close()

    @contextmanager
    def write_stream_and_check(self, filename, id_stream, boot_spec, id_agent, id_robot):
        """ This writes to a temporary file and checks everything is ok. """
        with safe_write_tmp_filename(filename, suffix_tmp='.tmp', suffix_old='.old') as tmp_filename:
            with self.write_stream(filename=tmp_filename,
                                              id_stream=id_stream,
                                              boot_spec=boot_spec,
                                              id_agent=id_agent,
                                              id_robot=id_robot) as writer:
                    yield writer        
        
            reader = LogsFormat.get_reader_for(filename)
            reader.index_file_cached(tmp_filename, ignore_cache=True)
        


LogsFormat.formats['h5'] = HDFLogsFormat2()
