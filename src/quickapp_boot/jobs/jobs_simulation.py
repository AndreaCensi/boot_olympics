from .jobs_parallel import get_tranches
from bootstrapping_olympics import get_conftools_robots
from bootstrapping_olympics.programs.manager.meat.simulate import (
    simulate_agent_robot)
from quickapp import ResourceManager
from quickapp.report_manager import basename_from_key
from quickapp_boot import RM_EPISODE_READY
from vehicles import VehicleSimulation

__all__ = ['recipe_episodeready_by_simulation_tranches',
           'recipe_episodeready_by_simulation']


def recipe_episodeready_by_simulation(context, data_central,
                                      id_robot, explorer, max_episode_len):
    """
        Provides episode-ready (id_robot, id_episode)
        
    """
    my_id_robot = id_robot
    
    def rp_simulate(c, id_robot, id_episode):
        if my_id_robot is not None and my_id_robot != id_robot:
            msg = ('I only know how to create %r, not %r.' % 
                   (my_id_robot, id_robot))
            raise ResourceManager.CannotProvide(msg)
        
        if not robot_supports_simulation(id_robot):
            msg = 'Robot %r is not simulatable.' % id_robot
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
    

def robot_supports_simulation(id_robot):
    robot = get_conftools_robots().instance(id_robot)
    orobot = robot.get_inner_components()[-1]
    if not isinstance(orobot, VehicleSimulation):
        return False
    else:
        return True

RM_SIMULATION_TRANCHES = 'tranches'

def recipe_episodeready_by_simulation_tranches(context, data_central, explorer, max_episode_len,
                                               episodes, episodes_per_tranche=50):
    """
        Provides episode-ready (id_robot, id_episode)
    
        
    """
    # TODO: add simulation-identifier
    
    tranches = get_tranches(episodes, episodes_per_tranche=episodes_per_tranche)
    episode2tranche = {}
    for i, episodes_per_tranche in enumerate(tranches):
        for e in episodes_per_tranche:
            episode2tranche[e] = i
            
    def rp_simulation_tranche(c, id_robot, tranche):
        tranche_episodes = tranches[tranche]
        job_id = basename_from_key(dict(id_robot=id_robot, explorer=explorer, tranche=tranche))
        return c.comp_config(simulate_agent_robot,
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
        
    print('recipe_episodeready_by_simulation_tranches: explorer %s max_episode_len %s' % (explorer, max_episode_len))
    rp_simulation_tranche.__docs__ = 'ciao'
        
    rm = context.get_resource_manager()        
    rm.set_resource_provider(RM_SIMULATION_TRANCHES, rp_simulation_tranche)

    def rp_episode_from_tranche(c, id_robot, id_episode):  # @UnusedVariable
        if not robot_supports_simulation(id_robot):
            msg = 'Robot %r is not simulatable.' % id_robot
            raise ResourceManager.CannotProvide(msg)

        if not id_episode in episode2tranche:
            msg = 'Episode %r not found for robot %r.' % (id_episode, id_robot)
            raise ResourceManager.CannotProvide(msg)
                
        t = episode2tranche[id_episode]
        return rm.get_resource(RM_SIMULATION_TRANCHES, id_robot=id_robot, tranche=t)

    rm = context.get_resource_manager()        
    rm.set_resource_provider(RM_EPISODE_READY, rp_episode_from_tranche)
#     
# 
# def recipe_tranche(context, data_central):
#     
#     def rp_simulation_tranches(c, id_robot, episodes, explorer, max_episode_len, episodes_per_tranche):
#         ntranches = len(tranches)
#         tranches_names = ['tranche%02d' % i for i in range(ntranches)]
#         
#         rm = c.get_resource_manager()
#     
#         for i, (c, name) in enumerate(iterate_context_names(c, tranches_names)):
#             tranche_episodes = tranches[i]
#             
#             job_id = basename_from_key(dict(id_robot=id_robot, explorer=explorer, tranche=name))
#             job = c.comp_config(simulate_agent_robot,
#                                 data_central=data_central,
#                                  id_agent=explorer,
#                                  id_robot=id_robot,
#                                  max_episode_len=max_episode_len,
#                                  stateful=False,
#                                  interval_print=5,
#                                  num_episodes=len(tranche_episodes),
#                                  id_episodes=tranche_episodes,
#                                  cumulative=False,
#                                  write_extra=False,
#                                  job_id=job_id)    
#             
#             for id_episode in tranche_episodes:
#                 rm.set_resource(job, RM_EPISODE_READY, id_robot=id_robot, id_episode=id_episode)
# 
#     rm = context.get_resource_manager()        
#     rm.set_resource_provider(RM_SIMULATION_TRANCHES, rp_simulation_tranches)
