from . import logger
from .. import BootStream
from ... import BootSpec, get_observations_dtype
from ...utils import yaml_load
import numpy as np
import tables

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
            id_agents = set(np.unique(table[:]['commands_source']))
            num_observations = len(table)
            bag_file = filename # TODO: change name
            topic = sid
            id_robot = table[0]['id_robot'] 
            timestamp = table[0]['timestamp']
            length = table[-1]['timestamp'] - table[0]['timestamp']
            stream = BootStream(id_robot, id_episodes, timestamp, length,
                                num_observations, bag_file, topic, spec,
                                id_agents)
            
            streams.append(stream)
        return streams
    finally:
        f.close()

    
def hdf_read(filename, id_stream, boot_spec, read_extra=False):
    f = tables.openFile(filename)
    try:
        # TODO: check table exists
        table = f.root.boot_olympics.streams._v_children[id_stream].boot_stream
        extra = f.root.boot_olympics.streams._v_children[id_stream].extra
        n = len(table)
        n_extra = len(extra)
        if n != n_extra:
            msg = ('In stream %s:%s I see %d observations, but only %d extra.' 
                   % (filename, id_stream, n, n_extra))
            logger.warn(msg)
        n = min(n, n_extra)
        dtype = get_observations_dtype(boot_spec)
        for i in range(n):
            row = table[i]
            observations = np.zeros((), dtype)
            for x in dtype.names:
                if x == 'extra': continue
                observations[x].flat = row[x].flat # FIXME Strange strange
            if read_extra:
                extra_string = str(extra[i])
                assert isinstance(extra_string, str)
                observations['extra'] = yaml_load(extra_string)
            else:
                observations['extra'] = {}
            yield observations
    finally:
        f.close()
