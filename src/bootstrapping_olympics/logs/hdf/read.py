from .. import BootStream
from ... import BootSpec
import numpy as np
import tables
from bootstrapping_olympics.interfaces.observations import get_observations_dtype
from bootstrapping_olympics.utils.c_yaml import yaml_load

def hdf_list_streams(filename):
    f = tables.openFile(filename)
    try:
        # TODO: check this has a valid format
        group = f.root.boot_olympics.streams
        ids = group._v_children.keys()
        
        streams = []
        for sid in ids:
            table = group._v_children[sid].boot_stream
            spec = BootSpec.from_yaml(yaml_load(str(table.attrs['boot_spec'])))
            id_episodes = set(np.unique(table[:]['id_episode']))
            num_observations = len(table)
            bag_file = filename # TODO: change name
            topic = sid
            id_robot = table[0]['id_robot'] 
            timestamp = table[0]['timestamp']
            length = table[-1]['timestamp'] - table[0]['timestamp']
            stream = BootStream(id_robot, id_episodes, timestamp, length,
                                num_observations, bag_file, topic, spec)
            
            streams.append(stream)
        return streams
    finally:
        f.close()

    
def hdf_read(filename, id_stream, boot_spec):
    f = tables.openFile(filename)
    try:
        # TODO: check table exists
        table = f.root.boot_olympics.streams._v_children[id_stream].boot_stream
        extra = f.root.boot_olympics.streams._v_children[id_stream].extra
        # TODO: read companion table with extra
        n = len(table)
        dtype = get_observations_dtype(boot_spec)
        for i in range(n):
            row = table[i]
            observations = np.zeros((), dtype)
            for x in dtype.names:
                if x == 'extra': continue
                observations[x].flat = row[x].flat # Strange strange
            observations['extra'] = yaml_load(str(extra[i]))
            yield observations
    finally:
        f.close()
