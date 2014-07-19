import os

from bootstrapping_olympics import EpisodeSummary, BootStream
from bootstrapping_olympics.utils import yaml_load
from streamels import BootSpec
from contracts import contract
import warnings

# from . import load_extra, tables


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
#         check_timed_named(x)
        _, (_, y) =x
        boot_info = yaml_load(y)
        
        print(boot_info)
        id_agent = boot_info['id_agent']
        id_robot = boot_info['id_robot']
        id_stream = boot_info['id_stream']
        
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
        
        print('read %.4f delta %.4f %s' % (time, time - timestamps[0], value))
        
    print('found episodes %r' % episodes)
    warnings.warn('reimplement')
    extras = []
    
    summaries = []
    for id_episode in episodes:
        summary = episode_summary(id_episode=id_episode,
                                  episodes=episodes,
                                  timestamps=timestamps, 
                                  id_agent=id_agent,
                                  id_robot=id_robot, extras=extras)
        summaries.append(summary)


    filename = os.path.realpath(filename)
    
    stream = BootStream(id_robot=id_robot,
                        filename=filename,
                        topic="what-topic?", 
                        spec=spec,
                        summaries=summaries)
    
    return [stream]

#     from hdflog import tables
#     f = tables.openFile(filename)
# #     try:
#         # TODO: check this has a valid format
#         group = f.root._v_children[PROCGRAPH_LOG_GROUP]
#         
#         # read the whole id_episode table
#         if not 'id_episode' in group._v_children:
#             msg = 'You did not log "id_episode" stream.'
#             raise Exception(msg)
# #         
#         id_episode_table = group.id_episode
#         extra_table = group.extra
#         episodes = set()
#         for row in id_episode_table:
#             episodes.add(row['value']) 
#         
        
#         spec = spec_from_group(group)
#         id_robot = group._v_attrs['id_robot']
#         id_agent = group._v_attrs['id_agent']
#         id_stream = group._v_attrs['id_stream']
#         
#     
#     except (ValueError, AttributeError):
#         logger.error('Error while trying to index %r.' % filename)
#         raise
#     finally:
#         f.close()

# 
# def spec_from_group(group):
#     specs = str(group.boot_spec[0])
#     spec = BootSpec.from_yaml(yaml_load(specs))
#     return spec
import numpy as np
@contract(id_episode='str', episodes='list[N]', timestamps='list[N]', 
          id_agent='str', id_robot='str')
def episode_summary(id_episode,
                      episodes,
                      timestamps, 
                      id_agent,
                      id_robot, extras):
    # XXX: do it differently when format change
    print('episode_summary(%s)' % id_episode)
#     print(id_episode_table)
#     print(id_episode_table[0])

    timestamps = np.array(timestamps)
#     print('timestamps: %s' % timestamps.dtype)
#     sel = [x == id_episode for x in episodes]
#     assert sel
#     print('selection : %s' % sel)
    
    
    stream = []
    for i, x in enumerate(episodes):
        if x == id_episode:
            stream.append(timestamps[i])
    
    
    num_observations = len(stream)
    if num_observations == 1:
        raise Exception('Episode %r is too short' % id_episode)
    
    for i, k in enumerate(stream):
        print('%.3f' % k)
    assert len(stream) >= 2
    t0 = stream[0]
    t1 = stream[-1]
    length = t1 - t0
    print('t0, t1: %.4f, %.4f = %.4f' % (t0, t1, length))
    
    id_world = 'what-id_world'
    
#     timestamp = stream[0]['timestamp']

#     first_moment = np.nonzero(sel)[0][0]
    # there might be different lengths of extra_table and sel
    # print len(extra_table)
    # print len(sel)

#     extra = load_extra(extra_table, 0)
#     extras = list(extra.keys())
#     

    return EpisodeSummary(id_episode=id_episode,
                          id_agent=id_agent,
                          id_world=id_world,
                          num_observations=num_observations,
                          length=length,
                          extras=extras, 
                          timestamp=t0)

