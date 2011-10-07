from . import BootStream, logger, LogsFormat
from collections import defaultdict
from conf_tools import locate_files
import os
import pickle

class LogIndex:
    def __init__(self):
        # id robot -> list of streams
        self.robots2streams = {}
        # filename -> list of streams for filename
        self.file2streams = {} 

    def index(self, directory, ignore_cache=False):
        new_streams = index_directory_cached(directory, ignore_cache)
        self.file2streams.update(new_streams)
        self.robot2streams = index_robots(self.file2streams)

def index_directory_cached(directory, ignore_cache=False):
    ''' Returns dict: filename -> list of BootStreams'''
    index_dir = os.path.join(directory, '.log_learn_indices')
    if not os.path.exists(index_dir):
        os.makedirs(index_dir)
    
    index_file = os.path.join(index_dir, 'index.pickle')
    
    needs_recreate = False
    
    if not os.path.exists(index_file):
        logger.debug('Index file not existing -- will create.')
        needs_recreate = True
    elif ignore_cache:
        logger.debug('Ignoring existing cache')
        needs_recreate = True
    elif os.path.getmtime(directory) > os.path.getmtime(index_file):
        # TODO: all subdirs
        logger.debug('Index file existing, but new logs added.')
        needs_recreate = True
        
    if needs_recreate:
        if os.path.exists(index_file):
            os.unlink(index_file)
        try:
            file2streams = index_directory(directory)
            for x, k in file2streams.items():
                assert isinstance(x, str)
                assert isinstance(k, list)
                
        
            with open(index_file, 'wb') as f:
                pickle.dump(file2streams, f)
        except:
            logger.error('Caught exception while indexing, deleting db.')
            if os.path.exists(index_file):
                os.unlink(index_file)
            raise
    else:
        logger.debug('Using cached index %r.' % index_file)
        
    try:
        with open(index_file, 'rb') as f:
            return pickle.load(f)
    except:
        logger.error('Index file corrupted; try deleting %r.' % (index_file))
        raise
    

def index_directory(directory):
    ''' Returns a hash filename -> list of streams. '''
    extensions = LogsFormat.formats.keys()
    
    files = []
    for extension in extensions:
        pattern = '*.%s' % extension
        files.extend(locate_files(directory, pattern))
    
    if not files:
        msg = ('No log files found in %r (extensions: %s).' % 
               (directory, extensions))
        logger.error(msg)

    file2streams = {}
    for i, filename in enumerate(files):
        logger.debug('%4d/%d: %s' % (i + 1 , len(files), filename))
        reader = LogsFormat.get_reader_for(filename)
        streams = reader.index_file_cached(filename) 
        if streams:
            for stream in streams:
                assert isinstance(stream, BootStream)
                logger.info('filename: %s stream: %s' % (filename, stream))
                logger.debug('%s: %s' % (stream.topic, stream))
            file2streams[filename] = streams
        else:
            logger.warning('No streams found. ')   
    return file2streams
    
def index_robots(file2streams):
    ''' Groups the streams by robot, making sure the specs are compatible. 
        Returns dict: id_robot -> list of streams.
    '''
    robot2streams = defaultdict(lambda:[])
    robot2spec = {}
    for _, streams in file2streams.items():
        for stream in streams:
            id_robot = stream.id_robot
            if not id_robot in robot2spec:
                robot2spec[id_robot] = stream.spec
            else:
                if str(stream.spec) != str(robot2spec[id_robot]):
                    msg = 'Warning! You got your logs mixed up. \n'
                    msg += ('Problem spec in:\n\t%s\nis\n\t%s\n' % 
                           (stream, stream.spec))
                    msg += ('and this is different from:\n\t%s\n'
                           'found in e.g.,:\n\t%s' % 
                           (robot2spec[id_robot], robot2streams[id_robot][0]))
                    msg += '\nI will skip this stream.'
                    logger.error(msg)
                    continue
            robot2streams[id_robot].append(stream)
    
    for robot in robot2streams:        
        robot2streams[robot] = sorted(robot2streams[robot],
                                 key=lambda x: list(x.id_episodes)[0])

    return robot2streams
    