from . import logger, np
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
                    spec = boot_spec_from_ros_message(msg)
                
                try:
                    commands = np.array(msg.commands)
                    spec.get_commands().check_valid_value(commands)
                except Exception as e:
                    logger.error('Invalid observations at #%d:%s' % (num, t))
                    logger.error(e)
                
                try:
                    observations = np.array(msg.sensel_values)
                    spec.get_observations().check_valid_value(observations)
                    # XXX: reshape first?
                    # spec.check_compatible_raw_sensels_values(msg.sensel_values)
                except Exception as e:
                    logger.error('Invalid commands at #%d:%s' % (num, t))
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


def boot_spec_from_ros_message(msg):
    ''' Infers the boot spec from a ROS message. '''
    commands_spec = eval(msg.commands_spec) # XXX: not safe 
    sensels_shape = msg.sensel_shape # TODO: check coherence

    nu = len(commands_spec)
    spec = {
            'observations': {
                             'shape': list(sensels_shape),
                             'range': [0, 1],
                             'format': 'C'
            },
            'commands': {
                         'shape': [nu],
                         'range': map(list, commands_spec),
                         'format': ['C'] * nu 
            }
    }
    return BootSpec.from_yaml(spec)
    
