from .jobs_parallel import get_tranches
from bootstrapping_olympics.programs.manager.meat.simulate import (
    simulate_agent_robot)
from quickapp import ResourceManager, iterate_context_names
from quickapp.report_manager import basename_from_key
from quickapp_boot import RM_EPISODE_READY

__all__ = ['recipe_episodeready_by_simulation_tranches',
           'recipe_episodeready_by_simulation']


def recipe_episodeready_by_simulation(context, data_central, id_robot, explorer, max_episode_len):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    my_id_robot = id_robot
    
    def rp_simulate(c, id_robot, id_episode):
        if my_id_robot is not None and my_id_robot != id_robot:
            msg = ('I only know how to create %r, not %r.' % 
                   (my_id_robot, id_robot))
            raise ResourceManager.CannotProvide(msg)
        
        job = c.comp_config(simulate_agent_robot,
                            data_central=data_central,
                             id_agent=explorer,
                             id_robot=id_robot,
                             max_episode_len=max_episode_len,
                             stateful=False,
                             interval_print=5,
                             num_episodes=1,
                             id_episodes=[id_episode],
                             cumulative=False,
                             write_extra=False)    
        return job
    
    rm = context.get_resource_manager()        
    rm.set_resource_provider(RM_EPISODE_READY, rp_simulate)
    

def recipe_episodeready_by_simulation_tranches(context, data_central, id_robot, explorer, max_episode_len,
                                               episodes, episodes_per_tranche=50):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    tranches = get_tranches(episodes, episodes_per_tranche=episodes_per_tranche)
    ntranches = len(tranches)
    tranches_names = ['tranche%02d' % i for i in range(ntranches)]
    
#     episode2job = {}
    
    rm = context.get_resource_manager()
    
    for i, (c, name) in enumerate(iterate_context_names(context, tranches_names)):
        tranche_episodes = tranches[i]
        
        job_id = basename_from_key(dict(id_robot=id_robot, explorer=explorer, tranche=name))
        job = c.comp_config(simulate_agent_robot,
                            data_central=data_central,
                             id_agent=explorer,
                             id_robot=id_robot,
                             max_episode_len=max_episode_len,
                             stateful=False,
                             interval_print=5,
                             num_episodes=len(tranche_episodes),
                             id_episodes=tranche_episodes,
                             cumulative=False,
                             write_extra=False,
                             job_id=job_id)    
        
        for id_episode in tranche_episodes:
#             episode2job[id_episode] = job

            rm.set_resource(job, RM_EPISODE_READY, id_robot=id_robot, id_episode=id_episode)
#             
#     my_id_robot = id_robot
#     
#     def rp_simulate(c, id_robot, id_episode):
#         if  my_id_robot != id_robot:
#             msg = ('I only know how to create %r, not %r.' % 
#                    (my_id_robot, id_robot))
#             raise ResourceManager.CannotProvide(msg)
#         
#         return episode2job[id_episode]
#     
#             
#     rm.set_resource_provider(RM_EPISODE_READY, rp_simulate)


# 
# @contract(returns='list(str)')
# def jobs_simulate(context, id_robot, explorer, episodes, max_episode_len=10):
#     """ Returns the episodes id """
#     # Divide the simulation in parallel tranches
#     
#     for c, id_episode in iterate_context_names():
#         tranche = context.comp(simulate_agent_robot,
#                         data_central=data_central,
#                          id_agent=explorer,
#                          id_robot=id_robot,
#                          max_episode_len=max_episode_len,
#                          stateful=False,
#                          interval_print=5,
#                          num_episodes=len(id_episodes),
#                          id_episodes=id_episodes,
#                          cumulative=False,
#                          write_extra=False,
#                          job_id='explore-%s-%s-%sof%s' % 
#                                 (id_robot, explorer, t + 1,
#                                  len(episodes_tranches)))
# 
#         tranches.append(tranche)
# 
# #         for id_episode in id_episodes:
# #             self.set_dep_episode_done(id_robot=id_robot,
# #                                       id_episode=id_episode, job=tranche)
# 
# #     self.add_videos(id_agent=explorer, id_robot=id_robot,
# #                     id_episodes=id_episodes_with_extra, videos=videos)
# 
#     return all_id_episodes
# 
# 
# import numpy as np
# 
# def get_tranches(ids, episodes_per_tranche=10):
#     """ Returns a list of list """
#     l = []
#     num_tranches = int(np.ceil(len(ids) * 1.0 / episodes_per_tranche))
#     for t in range(num_tranches):
#         e_from = t * episodes_per_tranche
#         e_to = min(len(ids), e_from + episodes_per_tranche)
#         l.append([ids[i] for i in range(e_from, e_to)])
#     return l
