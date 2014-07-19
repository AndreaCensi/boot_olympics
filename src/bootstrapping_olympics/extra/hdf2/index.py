from bootstrapping_olympics import BootStream, EpisodeSummary
from bootstrapping_olympics.utils import yaml_load
from contracts import contract
from streamels import BootSpec
import numpy as np
import os
import warnings

 
__all__ = [
    'hdf_list_streams',
]


def hdf_list_streams(filename):
    
    from hdflog.hdflogreader import PGHDFLogReader
    reader = PGHDFLogReader(filename)
    signals = reader.get_all_signals()
    if not 'boot_info':
        msg = 'Not a valid boot log: %s' % signals
        raise ValueError(msg)
    
    for x in reader.read_signal('boot_info'):
        _, (_, y) =x
        boot_info = yaml_load(y)
        
        print(boot_info)
        id_agent = boot_info['id_agent']
        id_robot = boot_info['id_robot']
        #id_stream = boot_info['id_stream']
        
        spec = BootSpec.from_yaml(boot_info['boot_spec'])
        break
    else:
        raise ValueError()
    
    timestamps =[]
    episodes =[]
    for time, (s, value) in reader.read_signal('id_episode'):
        assert s == 'id_episode'
        timestamps.append(time) 
        episodes.append(value)
        
    warnings.warn('reimplement')
    extras = []
    
    summaries = []
    for id_episode in episodes:
        summary = episode_summary(id_episode=id_episode,
                                  episodes=episodes,
                                  timestamps=timestamps, 
                                  id_agent=id_agent,  extras=extras)
        summaries.append(summary)


    filename = os.path.realpath(filename)
    
    stream = BootStream(id_robot=id_robot,
                        filename=filename,
                        topic="what-topic?", 
                        spec=spec,
                        summaries=summaries)
    
    return [stream]
 
@contract(id_episode='str', episodes='list[N]', 
          timestamps='list[N]', id_agent='str')
def episode_summary(id_episode,
                      episodes,
                      timestamps, 
                      id_agent,
                      extras):

    timestamps = np.array(timestamps)
    
    stream = []
    for i, x in enumerate(episodes):
        if x == id_episode:
            stream.append(timestamps[i])
    
    
    num_observations = len(stream)
    if num_observations == 1:
        raise Exception('Episode %r is too short' % id_episode)
    
    assert len(stream) >= 2
    t0 = stream[0]
    t1 = stream[-1]
    length = t1 - t0
    
    id_world = 'what-id_world'
     

    return EpisodeSummary(id_episode=id_episode,
                          id_agent=id_agent,
                          id_world=id_world,
                          num_observations=num_observations,
                          length=length,
                          extras=extras, 
                          timestamp=t0)

