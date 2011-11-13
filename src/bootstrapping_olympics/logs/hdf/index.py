from . import logger, np, tables
from .. import BootStream
from ... import BootSpec
from ...utils import yaml_load
import os
from bootstrapping_olympics.logs.boot_stream import EpisodeSummary

__all__ = ['hdf_list_streams', 'spec_from_table']


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
            spec = spec_from_table(table)
            id_episodes = set(np.unique(table[:]['id_episode']))
            
            summaries = [ episode_summary(table, extra, id_episode) 
                                for id_episode in id_episodes ]
                
            topic = sid
            id_robot = table[0]['id_robot'] 
            
            filename = os.path.realpath(filename)

            stream = BootStream(id_robot=id_robot,
                                filename=filename,
                                topic=topic, spec=spec,
                                summaries=summaries)
            
            streams.append(stream)
        return streams
    except (ValueError, AttributeError):
        logger.error('Error while trying to index %r.' % filename) 
        raise
    finally:
        f.close()

def spec_from_table(table):
    ''' Loads the spec from the boot_stream table. '''
    spec = BootSpec.from_yaml(yaml_load(str(table.attrs['boot_spec'])))
    return spec


def episode_summary(boot_stream, extra_table, id_episode):
    # XXX: do it differently when format change
    sel = boot_stream[:]['id_episode'] == id_episode
    stream = boot_stream[sel]
    
    id_agent = stream[-1]['commands_source'] # XXX
    id_world = stream[0]['id_world']
    length = stream[-1]['timestamp'] - stream[0]['timestamp']
    num_observations = len(stream) 
    timestamp = stream[0]['timestamp']
     
    extra_string = str(extra_table[sel][0])
    extra = yaml_load(extra_string)
    extras = list(extra.keys())
    return EpisodeSummary(id_episode=id_episode,
                          id_agent=id_agent,
                          id_world=id_world,
                          num_observations=num_observations,
                          length=length,
                          extras=extras,
                          timestamp=timestamp)
      
