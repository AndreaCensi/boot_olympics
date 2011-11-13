from . import BootStream, logger, LogsFormat, contract
from ..utils import natsorted
from collections import defaultdict
from conf_tools import locate_files 
import traceback

__all__ = ['LogIndex']

class LogIndex:
    def __init__(self):
        # id robot -> list of streams
        self.robots2streams = {}
        # filename -> list of streams  
        self.file2streams = {} 

        self.directories_indexed = set()
        
    def reindex(self):
        for dirname in self.directories_indexed:
            new_streams = index_directory_cached(dirname,
                                                ignore_cache=False,
                                                 #ignore_dir_cache=True
                                                 )
            self.file2streams.update(new_streams)
        self.robots2streams = index_robots(self.file2streams)
        
    def index(self, directory, ignore_cache=False):
        new_streams = index_directory_cached(directory,
                                             ignore_cache=ignore_cache,
                                             #ignore_dir_cache=True
                                             ) # XXX
        self.file2streams.update(new_streams)
        self.directories_indexed.add(directory)
        self.robots2streams = index_robots(self.file2streams)
        
    def has_streams_for_robot(self, id_robot):
        return id_robot in self.robots2streams
    
    @contract(returns='list')
    def get_streams_for_robot(self, id_robot):
        if not id_robot in self.robots2streams:
            raise ValueError('No streams for robot %r.' % id_robot)
        return self.robots2streams[id_robot]

    @contract(returns='list')
    def get_streams_for(self, id_robot, id_agent):
        if not id_robot in self.robots2streams:
            raise ValueError('No streams for robot %r.' % id_robot)
        streams = []
        for stream in self.robots2streams[id_robot]:
            if id_agent in stream.get_id_agents():
                streams.append(stream)
        return streams

    
    def get_robot_spec(self, id_robot):
        ''' Returns the spec of the robot as stored in the files. '''
        return self.robots2streams[id_robot][0].get_spec()
    
    def get_episodes_for_robot(self, id_robot, id_agent=None):
        ''' Returns a list of all episodes for the given robot (and
            agent if it is given). ''' # TODO: implement this
        episodes = []
        for stream in self.get_streams_for_robot(id_robot):
            if id_agent is None:
                episodes.extend(stream.get_id_episodes())
            else:
                if id_agent in stream.get_id_agents():
                    episodes.extend(stream.get_id_episodes())
        return natsorted(episodes)
    
    def read_all_robot_streams(self, id_robot, id_agent=None, read_extra=False):
        ''' Reads all the data corresponding to a robot.
            If agent is not None, it filters by agent.
         '''
        for stream in self.get_streams_for_robot(id_robot):
            if id_agent is None:
                do_this = True
            else:
                do_this = id_agent in stream.get_id_agents()
            if not do_this: continue
            
            for obs in stream.read(read_extra=read_extra):
                    yield obs 
            
    def read_robot_episode(self, id_robot, id_episode, read_extra=False):
        ''' Reads only one episode. '''
        for stream in self.get_streams_for_robot(id_robot):
            if id_episode in stream.get_id_episodes():
                for obs in stream.read(read_extra=read_extra):
                    if obs['id_episode'].item() == id_episode:
                        yield obs
                break
        else:
            msg = 'found:\n'
            for stream in self.get_streams_for_robot(id_robot):
                msg += ' %s: %s\n' % (stream, stream.get_id_episodes()) 
            raise Exception('No episode %r found: %s' % (id_episode, msg))
             


def index_directory(directory, ignore_cache=False):
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
    for   filename in files:
        reader = LogsFormat.get_reader_for(filename)
        try:
            file2streams[filename] = \
                reader.index_file_cached(filename, ignore_cache=ignore_cache) 
            for stream in file2streams[filename]:
                assert isinstance(stream, BootStream)
            if not file2streams[filename]:
                logger.warning('No streams found in file %r.' % filename)
        except None: # XXX
            logger.error('Invalid data in file %r.' % filename)
            logger.error(traceback.format_exc())
               
    return file2streams
    
def index_robots(file2streams):
    ''' Groups the streams by robot, making sure the specs are compatible. 
        Returns dict: id_robot -> list of streams.
    '''
    robot2streams = defaultdict(lambda:[])
    robot2spec = {}
    for _, streams in file2streams.items():
        for stream in streams:
            id_robot = stream.get_id_robot()
            
            if not id_robot in robot2spec:
                robot2spec[id_robot] = stream.get_spec()
            else:
                # XXX:
                stream_spec = stream.get_spec()
                if str(stream_spec) != str(robot2spec[id_robot]):
                    msg = 'Warning! You got your logs mixed up. \n'
                    msg += ('Problem spec in:\n\t%s\nis\n\t%s\n' % 
                           (stream, stream_spec))
                    msg += ('and this is different from:\n\t%s\n'
                           'found in e.g.,:\n\t%s' % 
                           (robot2spec[id_robot], robot2streams[id_robot][0]))
                    msg += '\nI will skip this stream.'
                    logger.error(msg)
                    continue
            robot2streams[id_robot].append(stream)
    
    for robot in robot2streams:        
        robot2streams[robot] = sorted(robot2streams[robot],
                                 key=lambda x: list(x.get_id_episodes())[0])
    
    return dict(**robot2streams)
    

index_directory_cached = index_directory

#def index_directory_cached(directory, ignore_dir_cache=False,
#                           ignore_file_cache=False):
#    ''' Returns dict: filename -> list of BootStreams'''
#    index_dir = os.path.join(directory, '.log_learn_indices')
#    if not os.path.exists(index_dir):
#        os.makedirs(index_dir)
#    
#    index_file = os.path.join(index_dir, 'index.pickle')
#    
#    needs_recreate = False
#    
#    logger.debug('Indexing %s %s' % (directory, ''))
#    if not os.path.exists(index_file):
#        #logger.debug('Index file not existing -- will create.')
#        needs_recreate = True
#    elif ignore_dir_cache:
#        #logger.debug('Ignoring existing cache')
#        needs_recreate = True
#    elif os.path.getmtime(directory) > os.path.getmtime(index_file):
#        # TODO: all subdirs
#        #logger.debug('Index file existing, but new logs added.')
#        needs_recreate = True
#        
#    if needs_recreate:
#        if os.path.exists(index_file):
#            os.unlink(index_file)
#        try:
#            file2streams = index_directory(directory,
#                                           ignore_cache=ignore_file_cache)
#            for x, k in file2streams.items():
#                assert isinstance(x, str)
#                assert isinstance(k, list)
#                
#            if not ignore_dir_cache:
#                with open(index_file, 'wb') as f:
#                    pickle.dump(file2streams, f)
#            
#            return file2streams
#        except:
#            logger.error('Caught exception while indexing, deleting db.')
#            if os.path.exists(index_file):
#                os.unlink(index_file)
#            raise
#    else:
#        logger.debug('Using cached index %r.' % index_file)
#            
#        try:
#            with open(index_file, 'rb') as f:
#                return pickle.load(f)
#        except:
#            logger.error('Index file corrupted; try deleting %r.' % (index_file))
#            raise
#        
