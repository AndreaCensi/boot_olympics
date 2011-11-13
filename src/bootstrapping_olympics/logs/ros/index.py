from . import logger, np
from .. import BootStream
from ... import BootSpec
import os
from bootstrapping_olympics.interfaces.stream_spec import BootInvalidValue
from bootstrapping_olympics.logs.boot_stream import EpisodeSummary
import warnings
from bootstrapping_olympics.utils.c_yaml import yaml_load


#def episode_summary(boot_stream, extra_table, id_episode):
#    # XXX: do it differently when format change
#    sel = boot_stream[:]['id_episode'] == id_episode
#    stream = boot_stream[sel]
#    
#    id_agent = stream[-1]['commands_source'] # XXX
#    id_world = stream[0]['id_world']
#    length = stream[-1]['timestamp'] - stream[0]['timestamp']
#    num_observations = len(stream) 
#    
#    extra_string = str(extra_table[sel][0])
#    extra = yaml_load(extra_string)
#    extras = list(extra.keys())
#    return EpisodeSummary(id_episode, id_agent, id_world, num_observations,
#                           length, extras)
#    

def bag_describe_stream(bag_file, bag, topic):
    
    timestamp = None
    spec = None
    num = 0
    num_invalid_obs = 0
    num_invalid_cmd = 0
    
    episode2summary = {}
    summaries = []
    
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if timestamp is None:
            timestamp = t
            
        if not msg.commands_source:
            warnings.warn('hack: skipping empty commands_source.')
            continue
        
        id_robot = msg.id_robot
        id_episode = msg.id_episode
        # XXX temporary hack, add warning
        if id_episode == 'id-episode-not-set':
            warnings.warn('hack: auto-assigning id_episode (bug in first versions '
                          'of BootOlympics)')
            id_episode = os.path.basename(bag_file)
            
        
        if spec is None:
            spec = boot_spec_from_ros_message(msg)
        
        if id_episode not in episode2summary:    
            summary = EpisodeSummary(id_episode=id_episode,
                         id_agent=msg.commands_source,
                         id_world=msg.id_environment,
                         num_observations=1000, # XXX
                         length=10, # XXX 
                         extras=[],
                         timestamp=msg.timestamp)
            episode2summary [id_episode] = summary
            summaries.append(summary)

        
        commands = np.array(msg.commands)
        try:
            spec.get_commands().check_valid_value(commands)
        except BootInvalidValue as e:
            if num_invalid_cmd <= 2:
                logger.error('Invalid observations at #%d:%s:\n%s' % (num, t, e))
            num_invalid_cmd += 1
        
        observations = np.array(msg.sensel_values)
        try:
            spec.get_observations().check_valid_value(observations)
            # XXX: reshape first?
            # spec.check_compatible_raw_sensels_values(msg.sensel_values)
        except BootInvalidValue as e:
            if num_invalid_obs <= 2:
                logger.error('Invalid commands at #%d:%s:\n%s' % (num, t, e))
            num_invalid_obs += 1
        
        num += 1
        
    if num_invalid_obs > 0:
        logger.error('Found %d invalid observations.' % num_invalid_obs)
    if num_invalid_cmd > 1:
        logger.error('Found %d invalid commands.' % num_invalid_cmd)
        
#    length = (t - timestamp)
#    length = float(int(length.to_sec()))
#    timestamp = float(int(timestamp.to_sec()))
#    
    stream = BootStream(id_robot=id_robot,
                        filename=bag_file,
                        topic=topic, spec=spec,
                        summaries=summaries)
  
#          
#    stream = BootStream(id_robot=id_robot,
#                                timestamp=timestamp,
#                                length=length,
#                                num_observations=num,
#                                bag_file=bag_file,
#                                topic=topic,
#                                spec=spec,
#                                summaries=summaries)
    return stream



def bag_get_bootstrapping_stream(bag_file):
    ''' 
        Returns the list of topics which are bootstrapping streams 
        in a bag file.
    '''
    boot_datatype = 'bootstrapping_adapter/BootstrappingObservations' # add in constants
    from ros import rosbag #@UnresolvedImport
    
    streams = []
    bag = rosbag.Bag(bag_file)
    topics = sorted(set([c.topic for c in bag._get_connections()]))
    for topic in topics:
        connections = list(bag._get_connections(topic))
        datatype = connections[0].datatype
        if datatype == boot_datatype:
            stream = bag_describe_stream(bag_file, bag, topic)
            streams.append(stream)

    bag.close()
    return streams


def boot_spec_from_ros_message(msg):
    ''' Infers the boot spec from a ROS message. '''
    
    # Previously, we used to write only the range as a string
    # for example, '[[0,1],[0,1],[0,1]]'
    # But now, we use the YAML spec directly.
    spec_string = msg.commands_spec
    
    if spec_string[0] == '[':
        commands_spec = eval(spec_string) # XXX: not safe 
        sensels_shape = msg.sensel_shape # TODO: check coherence
    
        nu = len(commands_spec)
        spec = {
                'observations': {
                     'shape': list(sensels_shape),
                     'range': [0, 1], # XXX: we need something else here
                     'format': 'C'
                },
                'commands': {
                     'shape': [nu],
                     'range': map(list, commands_spec),
                     'format': ['C'] * nu 
                }
        }
        return BootSpec.from_yaml(spec)
    elif spec_string[0] == '{':
        return BootSpec.from_yaml(yaml_load(spec_string))
        
    else:
        raise ValueError('Cannot interpret spec for msg:\n%s' % spec_string)
