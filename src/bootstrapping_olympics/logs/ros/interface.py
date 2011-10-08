from . import bag_get_bootstrapping_stream, bag_read
from .. import LogsFormat

class ROSLogsFormat(LogsFormat):
    
    def index_file(self, filename):
        ''' Returns a list of BootStream objects. '''
        return bag_get_bootstrapping_stream(filename)
    
    def read_stream(self, stream):
        ''' Yields observations from the stream. '''
        for x in bag_read(stream.bag_file,
                          stream.topic,
                          stream.spec,
                          substitute_id_episode=stream.short_file):
            yield x

    

LogsFormat.formats['bag'] = ROSLogsFormat()
