from . import logger
from collections import defaultdict
from vehicles.configuration.config_utils import locate_files
import os
import shelve
from . import BootStream 


def bag_get_index_object(directory):
    index_dir = os.path.join(directory, '.log_learn_indices')
    if not os.path.exists(index_dir):
        os.makedirs(index_dir)
    
    index_file = os.path.join(index_dir, 'index')
    
    needs_recreate = False
    
    if not os.path.exists(index_file):
        logger.debug('Index file not existing -- will create.')
        needs_recreate = True
    elif os.path.getmtime(directory) > os.path.getmtime(index_file):
        logger.debug('Index file existing, but new logs added.')
        needs_recreate = True
        
    if needs_recreate:
        if os.path.exists(index_file):
            os.unlink(index_file)
        try:
            s = shelve.open(index_file, protocol=2, writeback=True)
            reindex(s, directory)
            bag_files = s['bag_files']
            robots = defaultdict(lambda:[])
            for file, topics in bag_files.items():
                for stream in topics.values():
                    robots[stream.id_robot].append(stream)
            s['robots'] = dict(**robots)
            s.close()
        except:
            logger.error('Caught exception while indexing, deleting db.')
            if os.path.exists(index_file):
                os.unlink(index_file)
            raise
    else:
        logger.debug('Using cached index %r.' % index_file)
        
    assert os.path.exists(index_file)
    try:
        s = shelve.open(index_file, writeback=True) 
        assert 'bag_files' in s
        assert 'robots' in s
        x = s['bag_files'] #try loading from disk @UnusedVariable
        x = s['robots'] #try loading from disk @UnusedVariable
    except:
        logger.error('Index file corrupted; try deleting %r.' % (index_file))
        raise
    return s
          
def reindex(s, directory):
    bags = list(locate_files(directory, '*.bag'))
    if not bags:
        logger.error('No bag files found in %r.' % directory)

    s['bag_files'] = {}
    for i, bag_file in enumerate(bags):
        logger.debug('%4d/%d: %s' % (i + 1 , len(bags), bag_file))
        streams = bag_get_bootstrapping_stream(bag_file)
        if streams:
            for topic, content in streams.items():
                logger.debug('%s: %s' % (topic, content))
            s['bag_files'][bag_file] = streams
        else:
            logger.info('  No bootstrapping data found. ')   


def bag_get_bootstrapping_stream(bag_file):
    ''' 
        Returns the list of topics which are bootstrapping streams 
        in a bag file.
    '''
    boot_datatype = 'bootstrapping_adapter/BootstrappingObservations'
    from ros import rosbag #@UnresolvedImport
    
    streams = {}
    bag = rosbag.Bag(bag_file)
    topics = sorted(set([c.topic for c in bag._get_connections()]))
    for topic in topics:
        connections = list(bag._get_connections(topic))
        datatype = connections[0].datatype
        if datatype == boot_datatype:
            id_episodes = set([])
            timestamp = None
            num = 0
            for topic, msg, t in bag.read_messages(topics=[topic]):
                if timestamp is None:
                    timestamp = t
                id_robot = msg.id_robot
                id_episode = msg.id_episode
                # HACK
                if id_episode == 'id-episode-not-set':
                    id_episode = os.path.basename(bag_file)
                id_episodes.add(id_episode)
                num += 1
            length = (t - timestamp)
            length = float(int(length.to_sec()))
            timestamp = float(int(timestamp.to_sec()))
            streams[topic] = BootStream(id_robot=id_robot,
                                        id_episodes=id_episodes,
                                        timestamp=timestamp,
                                        length=length,
                                        num_observations=num,
                                        bag_file=bag_file,
                                        topic=topic)

    bag.close()
    return streams
    
