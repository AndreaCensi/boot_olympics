from .. import BootStream
from ... import BootSpec
import numpy as np
import tables
import yaml

def hdf_list_streams(filename):
    f = tables.openFile(filename)
    # TODO: check this has a valid format
    group = f.root.boot_olympics.logs
    ids = group._v_children.keys()
    
    streams = []
    for sid in ids:
        table = group._v_children[sid]
        id_episodes = set(np.unique(table[:]['id_episode']))
        num_observations = len(table)
        bag_file = filename # TODO: change name
        topic = sid
        id_robot = table[0]['id_robot']
        spec = BootSpec.from_yaml(yaml.load(table[0]['spec']))
        
        timestamp = table[0]['timestamp']
        length = table[-1]['timestamp'] - table[0]['timestamp']
        stream = BootStream(id_robot, id_episodes, timestamp, length,
                            num_observations, bag_file, topic, spec)
        
        streams.append(stream)
    f.close()
    return streams
    
def hdf_read(filename, id_stream):
    f = tables.openFile(filename)
    # TODO: check table exists
    table = f.root.boot_olympics.logs._v_children[id_stream]
    n = len(table)
    for i in range(n):
        yield np.array(table[i])
    
    f.close()
