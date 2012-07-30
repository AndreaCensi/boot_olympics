from . import BootStream, logger, LogsFormat, contract
from ..utils import natsorted
from ..utils import warn_long_time
from collections import defaultdict
from conf_tools import locate_files
from conf_tools.utils import friendly_path
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
            new_streams = \
                index_directory_cached(dirname, ignore_cache=False)
            self.file2streams.update(new_streams)
        self.robots2streams = index_robots(self.file2streams)

    def index(self, directory, ignore_cache=False):
        new_streams = \
            index_directory_cached(directory, ignore_cache=ignore_cache)
        self.file2streams.update(new_streams)
        self.directories_indexed.add(directory)
        self.robots2streams = index_robots(self.file2streams)

    def has_streams_for_robot(self, id_robot):
        return id_robot in self.robots2streams

    @contract(returns='list(str)')
    def list_robots(self):
        """ Returns a list of the robots """
        return list(self.robots2streams.keys())

    @contract(returns='list')
    def get_streams_for_robot(self, id_robot):
        if not id_robot in self.robots2streams:
            raise ValueError('No streams for robot %r; available %s.' % 
                             (id_robot, self.robots2streams.keys()))
        return self.robots2streams[id_robot]

    @contract(returns='list')
    def get_streams_for(self, id_robot, id_agent=None):
        if not id_robot in self.robots2streams:
            raise ValueError('No streams for robot %r.' % id_robot)
        streams = []
        for stream in self.robots2streams[id_robot]:
            if id_agent:
                if id_agent in stream.get_id_agents():
                    streams.append(stream)
            else:
                streams.append(stream)
        return streams

    def get_robot_spec(self, id_robot):
        ''' Returns the spec of the robot as stored in the files. '''
        return self.robots2streams[id_robot][0].get_spec()

    # TODO: implement has_extra
    def get_episodes_for_robot(self, id_robot, id_agent=None):
        ''' Returns a list of all episodes for the given robot (and
            agent if it is given). ''' # TODO: implement this
        episodes = []
        for stream in self.get_streams_for_robot(id_robot):
            if not id_agent:
                episodes.extend(stream.get_id_episodes())
            else:
                if id_agent in stream.get_id_agents():
                    episodes.extend(stream.get_id_episodes())
        return natsorted(episodes)

    # TODO: implement has_extra
    def read_all_robot_streams(self, id_robot,
                               id_agent=None, read_extra=False):
        ''' Reads all the data corresponding to a robot.
            If agent is not None, it filters by agent.
         '''
        for stream in self.get_streams_for_robot(id_robot):
            if not id_agent:
                do_this = True
            else:
                do_this = id_agent in stream.get_id_agents()
            if not do_this:
                continue

            for obs in stream.read(read_extra=read_extra):
                yield obs

    def read_robot_episode(self, id_robot, id_episode, read_extra=False):
        ''' Reads only one episode. '''
        for stream in self.get_streams_for_robot(id_robot):
            if id_episode in stream.get_id_episodes():

                for obs in stream.read(read_extra=read_extra,
                                       only_episodes=[id_episode]):
                    assert obs['id_episode'].item() == id_episode
                    yield obs

                break
        else:
            msg = 'found:\n'
            for stream in self.get_streams_for_robot(id_robot):
                msg += ' %s: %s\n' % (stream, stream.get_id_episodes())
            raise Exception('No episode %r found: %s' % (id_episode, msg))

    @contract(returns='str')
    def debug_summary(self, ignore_cache=False):
        ''' Returns a summary of the log files found in the directories. '''
        msg = 'debug_summary output (ignore_Cache=%s): \n' % ignore_cache
        for dirname in self.directories_indexed:
            msg += 'Dir %r: \n' % dirname
            for filename in get_all_log_files(dirname):
                reader = LogsFormat.get_reader_for(filename)
                streams = reader.index_file_cached(filename,
                                                   ignore_cache=ignore_cache)
                msg += '*  %5d streams in %s\n' % (len(streams), filename)
                for stream in streams:
                    msg += '  - stream %s \n' % stream
                    msg += '    id_robot: %r\n' % stream.get_id_robot()
                    msg += '    agents: %r\n' % stream.get_id_agents()
        return msg


def get_all_log_files(directory):
    ''' Returns all log files in the directory, for all registered
        extensions. '''
    extensions = LogsFormat.formats.keys()

    files = []
    for extension in extensions:
        pattern = '*.%s' % extension
        files.extend(locate_files(directory, pattern))

    if not files:
        msg = ('No log files found in %r (extensions: %s).' % 
               (friendly_path(directory), extensions))
        logger.warning(msg)

    return files


def index_directory(directory, ignore_cache=False, warn_if_longer=3):
    ''' Returns a hash filename -> list of streams. '''
    file2streams = {}
    logger.debug('Indexing directory %r (ignore cache: %s).' % 
                 (friendly_path(directory), ignore_cache))
    
    with warn_long_time(warn_if_longer, 'indexing directory %r' % 
                                        friendly_path(directory)):
        files = get_all_log_files(directory)
    
    # Shuffle the list so that multiple threads will index different files
    import random
    random.seed()
    random.shuffle(files)

    with warn_long_time(warn_if_longer, 'indexing %d files (use cache: %s)' % 
                        (len(files), not ignore_cache)):
        for filename in files:
            reader = LogsFormat.get_reader_for(filename)
            try:
                file2streams[filename] = \
                    reader.index_file_cached(filename, ignore_cache=ignore_cache)
                for stream in file2streams[filename]:
                    assert isinstance(stream, BootStream)
                if not file2streams[filename]:
                    logger.warning('No streams found in file %r.' % 
                                   friendly_path(filename))
            except None: # XXX
                logger.error('Invalid data in file %r.' % friendly_path(filename))
                logger.error(traceback.format_exc())

  
    return file2streams


def index_robots(file2streams):
    ''' Groups the streams by robot, making sure the specs are compatible. 
        Returns dict: id_robot -> list of streams.
    '''
    robot2streams = defaultdict(lambda: [])
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


index_directory_cached = index_directory # TODO: remove

