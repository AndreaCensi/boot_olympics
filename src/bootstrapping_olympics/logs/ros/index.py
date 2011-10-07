from . import logger
from .. import BootStream
from ... import BootSpec
import os

def bag_get_bootstrapping_stream(bag_file):
    ''' 
        Returns the list of topics which are bootstrapping streams 
        in a bag file.
    '''
    boot_datatype = 'bootstrapping_adapter/BootstrappingObservations'
    from ros import rosbag #@UnresolvedImport
    
    streams = []
    bag = rosbag.Bag(bag_file)
    topics = sorted(set([c.topic for c in bag._get_connections()]))
    for topic in topics:
        connections = list(bag._get_connections(topic))
        datatype = connections[0].datatype
        if datatype == boot_datatype:
            id_episodes = set([])
            timestamp = None
            num = 0
            spec = None
            for topic, msg, t in bag.read_messages(topics=[topic]):
                if timestamp is None:
                    timestamp = t
                id_robot = msg.id_robot
                id_episode = msg.id_episode
                # HACK
                if id_episode == 'id-episode-not-set':
                    id_episode = os.path.basename(bag_file)
                id_episodes.add(id_episode)
                if spec is None:
                    spec = BootSpec.from_ros_structure(msg)
                
                try:
                    spec.check_compatible_commands_values(msg.commands)
                    spec.check_compatible_raw_sensels_values(msg.sensel_values)
                except Exception as e:
                    logger.error('Invalid data at #%d:%s' % (num, t))
                    logger.error(e)
                
                num += 1
            length = (t - timestamp)
            length = float(int(length.to_sec()))
            timestamp = float(int(timestamp.to_sec()))
            stream = BootStream(id_robot=id_robot,
                                        id_episodes=id_episodes,
                                        timestamp=timestamp,
                                        length=length,
                                        num_observations=num,
                                        bag_file=bag_file,
                                        topic=topic,
                                        spec=spec)
            streams.append(stream)

    bag.close()
    return streams
