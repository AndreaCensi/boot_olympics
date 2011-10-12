'''Some functions to help in writing experiments scripts'''

from . import DataCentral
from .cmd_learn import learn_log, publish_once
from .cmd_simulate import simulate
from optparse import OptionParser
import contracts
import itertools


def experiment_explore_learn_main(proj_root,
                                  explorer, agents, robots,
                                  args):
    parser = OptionParser()
    parser.disable_interspersed_args()
    parser.add_option("--reset", default=False, action='store_true',
                      help="Reset the state of the agents.")
    parser.add_option("--compmake", default=False, action='store_true',
                      help="Uses compmake [%default]")
    parser.add_option("--write_extra", default=False, action='store_true',
                      help="Writes extra info in the logs (robot state) [%default]")
    parser.add_option("--num_episodes", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    (options, args) = parser.parse_args(args)
    
    
    contracts.disable_all() # TODO: option

    if options.compmake:
        from compmake import compmake_console #@UnresolvedImport
        experiment_explore_learn_compmake(proj_root,
                             explorer, agents, robots,
                             episode_len=options.episode_len,
                             num_episodes=options.num_episodes,
                             write_extra=True)
        compmake_console()
    else:
        experiment_explore_learn(proj_root,
                             explorer, agents, robots,
                             episode_len=options.episode_len,
                             num_episodes=options.num_episodes,
                             write_extra=True)


def experiment_explore_learn(proj_root,
                             explorer, agents, robots, episode_len,
                             num_episodes, write_extra=True,
                             reset=False):
    #from compmake import comp
        
    data_central = DataCentral(proj_root)
    
    index = data_central.get_log_index()
    
    for id_robot in robots:
        simulate(data_central=data_central,
                 id_agent=explorer,
                 id_robot=id_robot,
                 max_episode_len=episode_len,
                 num_episodes=num_episodes,
                 stateful=False,
                 interval_print=5,
                 cumulative=True,
                 write_extra=write_extra)
    
    index.reindex()
    
    
    for id_robot, id_agent  in itertools.product(robots, agents):
        learn_log(data_central=data_central,
                  id_agent=id_agent,
                  id_robot=id_robot,
                  reset=reset,
                  publish_interval=None,
                  publish_once=False,
                  interval_save=300,
                  interval_print=5)
    
        publish_once(data_central, id_agent, id_robot)
    


def experiment_explore_learn_compmake(proj_root,
                             explorer, agents, robots, episode_len,
                             num_episodes, write_extra=True,
                             reset=False):
    from compmake import comp #, compmake_storage @UnresolvedImport
    
    data_central = DataCentral(proj_root)
#    ds = data_central.get_directory_structure()
    # compmake_storage(ds.get_storage_dir()) # TODO
    
    robot2simulations = {}
    for id_robot in robots:
        job_id = 'simulate-%s' % (id_robot)
        robot2simulations[id_robot] = comp(simulate,
                                            data_central=data_central,
                                             id_agent=explorer,
                                             id_robot=id_robot,
                                             max_episode_len=episode_len,
                                             num_episodes=num_episodes,
                                             stateful=False,
                                             interval_print=5,
                                             cumulative=True,
                                             write_extra=write_extra,
                                             job_id=job_id)
        
        if write_extra: # also add video
            for i in range(num_episodes):
                job_id = 'video-exploration-%s-ep%04d' % (id_robot, i)  
                comp(create_video,
                     data_central=data_central,
                     episodes=robot2simulations[id_robot],
                     episode_index=i,
                     id_robot=id_robot,
                     job_id=job_id)

    # FIXME: needs redoing
    #index.reindex()
    
    ra2learned = {}
    for id_robot, id_agent in itertools.product(robots, agents):
        extra_dep = [robot2simulations[id_robot]]
        job_id = 'learn-%s-%s' % (id_robot, id_agent)
        ra2learned[(id_robot, id_agent)] = comp(
                            learn_log, data_central=data_central,
                  id_agent=id_agent,
                  id_robot=id_robot,
                  reset=reset,
                  publish_interval=None,
                  publish_once=False,
                  interval_save=300,
                  interval_print=5,
                  extra_dep=extra_dep,
                  job_id=job_id)
    
        job_id = 'publish-%s-%s' % (id_robot, id_agent)
        extra_dep = ra2learned[(id_robot, id_agent)]
        comp(publish_once, data_central, id_agent, id_robot,
             job_id=job_id, extra_dep=extra_dep)
    
            
def create_video(data_central, episodes, episode_index, id_robot):
    id_episode = episodes[episode_index]
    ds = data_central.get_dir_structure()
    filename = ds.get_video_filename(id_robot, id_episode)
    logdir = ds.get_simulated_logs_dir()
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_episode=id_episode,
                output=filename)
    pg('boot_log2movie', config=config)

