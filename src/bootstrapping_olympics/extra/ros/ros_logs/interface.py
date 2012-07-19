from . import BagLogWriter, bag_get_bootstrapping_stream, bag_read
from .. import boot_has_ros
from bootstrapping_olympics import LogsFormat
from contextlib import contextmanager
import os


class ROSLogsFormat(LogsFormat):

    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        return bag_get_bootstrapping_stream(filename)

    def read_stream(self, stream, read_extra=False, only_episodes=None):
        ''' Yields observations from the stream. '''
        if read_extra:
            raise ValueError('Cannot read extras in ROS format yet.')

        subst = os.path.splitext(os.path.basename(stream.get_filename()))[0]
        for x in bag_read(stream.get_filename(),
                          stream.get_topic(),
                          stream.get_spec(),
                          only_episodes=only_episodes,
                          substitute_id_episode=subst): # XXX
            yield x

    def read_from_stream(self, filename, id_stream, read_extra=False,
                                only_episodes=None):
        ''' Yields observations from the stream. '''

        if read_extra:
            raise ValueError('Cannot read extras in ROS format yet.')

        subst = os.path.splitext(os.path.basename(filename))[0]
        for x in bag_read(filename, id_stream, spec=None,
                          only_episodes=only_episodes,
                          substitute_id_episode=subst): # XXX
            yield x

    @contextmanager
    def write_stream(self, filename, id_stream, boot_spec):
        writer = BagLogWriter(filename, id_stream, boot_spec)
        try:
            yield writer
        except:
            writer.cleanup()
            raise
        else:
            writer.close()

if boot_has_ros:
    LogsFormat.formats['bag'] = ROSLogsFormat()
