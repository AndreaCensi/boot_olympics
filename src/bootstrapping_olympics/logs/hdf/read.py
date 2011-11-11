from . import logger, np, tables
from .. import BootStream
from ... import BootSpec, get_observations_dtype
from ...utils import yaml_load
from bootstrapping_olympics.logs.boot_stream import EpisodeSummary
import os
import time


__all__ = ['hdf_list_streams', 'hdf_read']

def hdf_list_streams(filename):
    f = tables.openFile(filename)
    try:
        # TODO: check this has a valid format
        group = f.root.boot_olympics.streams
        ids = group._v_children.keys()
        
        streams = []
        for sid in ids:
            table = group._v_children[sid].boot_stream
            extra = group._v_children[sid].extra
            spec = BootSpec.from_yaml(yaml_load(str(table.attrs['boot_spec'])))
            id_episodes = set(np.unique(table[:]['id_episode']))
            id_agents = set(np.unique(table[:]['commands_source']))
            
            episodes_summary = [ episode_summary(table, extra, id_episode) 
                                for id_episode in id_episodes ]
                
            num_observations = len(table)
            topic = sid
            id_robot = table[0]['id_robot'] 
            timestamp = table[0]['timestamp']
            length = table[-1]['timestamp'] - table[0]['timestamp']
            
            filename = os.path.realpath(filename)
            stream = BootStream(id_robot, id_episodes, timestamp, length,
                                num_observations, filename, topic, spec,
                                id_agents, episodes_summary)
            
            streams.append(stream)
        return streams
    finally:
        f.close()

def episode_summary(boot_stream, extra_table, id_episode):
    # XXX: do it differently when format change
    sel = boot_stream[:]['id_episode'] == id_episode
    stream = boot_stream[sel]
    
    id_agent = stream[-1]['commands_source'] # XXX
    id_world = stream[0]['id_world']
    length = stream[-1]['timestamp'] - stream[0]['timestamp']
    num_observations = len(stream) 
    
    extra_string = str(extra_table[sel][0])
    extra = yaml_load(extra_string)
    extras = list(extra.keys())
    return EpisodeSummary(id_episode, id_agent, id_world, num_observations,
                           length, extras)
      
               
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
#                t0 = time.clock()
                
                observations['extra'] = yaml_load(extra_string)
#                logger.debug('read string of len %s in %s' % (len(extra_string),
#                                                              time.clock() - t0))
            else:
                observations['extra'] = {}
            yield observations
    finally:
        f.close()
