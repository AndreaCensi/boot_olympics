'''Some functions to help in writing experiments scripts'''
from . import (task_predict, DataCentral, logger, learn_log, publish_once, np,
    task_servo)
from .meat import simulate
from optparse import OptionParser
import contracts
import os
import shutil
from bootstrapping_olympics.programs.manager.meat.servo_stats import servo_stats_report, \
    servo_stats_summaries

def experiment_explore_learn_main(proj_root,
                                  explorer, agents, robots,
                                  args):
    parser = OptionParser()
    parser.disable_interspersed_args()
    parser.add_option("--reset", default=False, action='store_true',
                      help="Reset the state of the agents.")
    parser.add_option("--resimulate", default=False, action='store_true',
                      help="Resimulates the logs.")
    parser.add_option("--republish", default=False, action='store_true',
                      help="Cleans the reports.")
    
#    parser.add_option("--write_extra", default=False, action='store_true',
#                      help="Writes extra info in the logs (robot state) [%default]")
    parser.add_option("--num_ep_expl", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--num_ep_expl_v", type='int', default=6,
                      help="Number of videos of exploration to create [%default]")
    parser.add_option("--num_ep_serv", type='int', default=0,
                      help="Number of servoing episodes to simulate [%default]")
    parser.add_option("--num_ep_serv_v", type='int', default=0,
                      help="Number of videos for servoing. [%default]")
    
    parser.add_option("--episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--contracts", default=False, action='store_true',
                      help="Slower, more checks.")
    parser.add_option("--only", default=None,
                      help="Only do things for this robot.")
    (options, args) = parser.parse_args(args)
    
    if not options.contracts:
        logger.info('Disabling PyContracts for maximum speed'
                    ' (use --contracts to enable).')
        contracts.disable_all()

    from compmake import compmake_console, batch_command #@UnresolvedImport
    data_central = DataCentral(proj_root)

    experiment_explore_learn_compmake(data_central, explorer, agents, robots,
                         episode_len=options.episode_len,
                         num_ep_expl=options.num_ep_expl,
                         num_ep_expl_v=options.num_ep_expl_v,
                         num_ep_serv=options.num_ep_serv,
                         num_ep_serv_v=options.num_ep_serv_v,
                         reset=options.reset
                         )
    
    if options.only:
        id_robot = options.only

    if options.reset:
        if options.only:
            batch_command('clean learn*X*'.replace('X', id_robot))
        else:
            batch_command('clean learn*')
        
    if options.republish:
        logger.info('Removing previous logs.')
        dirname = os.path.join(proj_root, 'reports', 'learn')
        if os.path.exists(dirname):
            shutil.rmtree(dirname)
            
        batch_command('clean publish*')
        
    if options.resimulate: 
        logger.info('Removing previous logs.')
        if options.only:
            dirname = os.path.join(proj_root, 'logs', 'simulations', id_robot)
        else:
            dirname = os.path.join(proj_root, 'logs', 'simulations')
            
        if os.path.exists(dirname):
            shutil.rmtree(dirname)
                
        batch_command('clean simulate*')
        
    if options.only:
        id_robot = options.only
        logger.info('Only doing things for robot %r.' % id_robot)
        cmd1 = 'make sim*X* pub*X* '.replace('X', id_robot)
        batch_command(cmd1)
        if options.write_extra:
            cmd2 = 'make video-exploration-X-ep0000-*'.replace('X', id_robot)
            batch_command(cmd2)
        
    compmake_console() 

def episode_id_exploration(K):
    return 'ep_expl_%05d' % K

def episode_id_servoing(K):
    return 'ep_serv_%05d' % K

def experiment_explore_learn_compmake(data_central,
                             explorer, agents, robots, episode_len,
                             num_ep_expl=10,
                             num_ep_expl_v=1,
                             num_ep_serv=1,
                             num_ep_serv_v=1,
                             servo_displacement=1,
                             servo_max_episode_len=5,
                             reset=False,
                             episodes_per_tranche=10):
    from compmake import comp
    
    if not robots: 
        raise Exception('Please specify at least one robot.')
    if not agents: 
        raise Exception('Please specify at least one agent.')
    
    logger.info('Creating set for robots: %s\n agents: %s' % (robots, agents))

    for id_robot in robots:
#        id_robot = BootOlympicsConfig.robots.id_from_spec(robot_spec)
        
        # Divide the simulation in parallel tranches
        num_tranches = int(np.ceil(num_ep_expl * 1.0 / episodes_per_tranche))
        tranches = []
        episode2tranche = {}
        tranchenum2episodes = {}
        
        for t in range(num_tranches):
            e_from = t * episodes_per_tranche
            e_to = min(num_ep_expl, e_from + episodes_per_tranche)
            t_write_extra = num_ep_expl_v > e_from
            tranchenum2episodes[t] = [episode_id_exploration(K) 
                                      for K in range(e_from, e_to)]
            tranche = comp(simulate,
                            data_central=data_central,
                             id_agent=explorer,
                             id_robot=id_robot,
                             max_episode_len=episode_len,
                             stateful=False,
                             interval_print=5,
                             num_episodes=(e_to - e_from),
                             id_episodes=tranchenum2episodes[t],
                             cumulative=False,
                             write_extra=t_write_extra,
                             job_id='simulate-%s-%s' % (id_robot, t + 1))
            tranches.append(tranche)
            for id_episode in tranchenum2episodes[t]:
                episode2tranche[id_episode] = tranche
 
        comp(checkpoint, 'all simulations',
                         job_id='simulate-%s' % (id_robot),
                         extra_dep=tranches)
         
        add_exploration_videos(data_central=data_central,
                               id_robot=id_robot,
                               id_agent=explorer,
                               episode2tranche=episode2tranche,
                               num_ep_expl_v=num_ep_expl_v)        
 
        for id_agent in agents:
            # Learn tranche by tranche
            previous_state = None
            for t, tranche in enumerate(tranches):
                reset = (t == 0)
                extra_dep = [tranche] 
                if previous_state:
                    extra_dep.append(previous_state)
                
                previous_state = comp(learn_log, data_central=data_central,
                                      id_agent=id_agent,
                                      id_robot=id_robot,
                                      reset=reset,
                                      episodes=tranchenum2episodes[t],
                                      publish_interval=None,
                                      publish_once=False,
                                      interval_save=300,
                                      interval_print=5,
                                      extra_dep=extra_dep,
                                      job_id='learn-%s-%s-%s' % (id_robot, id_agent, t))
    
                comp(publish_once, data_central, id_agent, id_robot,
                     phase='learn', progress='t%03d' % t,
                     job_id='publish-%s-%s-%s' % (id_robot, id_agent, t),
                     extra_dep=previous_state)
    
            all_learned = comp(checkpoint, 'all learned',
                                job_id='learn-%s-%s' % (id_robot, id_agent),
                                extra_dep=[previous_state])
                
            comp(publish_once, data_central, id_agent, id_robot,
                 phase='learn', progress='all',
                 job_id='publish-%s-%s' % (id_robot, id_agent),
                 extra_dep=all_learned)
    
            # ra2learned[(id_robot, id_agent)] = all_learned
            has_servo = agent_has_servo(data_central, id_agent) 
            
            if not has_servo:
                logger.debug('Agent %s does not support servoing.' % id_agent)
                   
            if has_servo and num_ep_serv > 0: 
                
                num_servo_tranches = int(np.ceil(num_ep_serv * 1.0 / episodes_per_tranche))
                all_tranches = []
                for st in range(num_servo_tranches):
                    e_from = st * episodes_per_tranche
                    e_to = min(num_ep_serv, e_from + episodes_per_tranche)
                    num_episodes_with_robot_state = max(0, num_ep_serv_v - e_from)
                    st_id_episodes = [episode_id_servoing(K) 
                                              for K in range(e_from, e_to)]
                    assert len(st_id_episodes) > 0
                    tranche = comp(task_servo, data_central=data_central,
                     id_agent=id_agent, id_robot=id_robot,
                     max_episode_len=servo_max_episode_len,
                     num_episodes=len(st_id_episodes),
                     id_episodes=st_id_episodes,
                     cumulative=False,
                     displacement=servo_displacement,
                     interval_print=5,
                     num_episodes_with_robot_state=num_episodes_with_robot_state,
                     job_id='servo-%s-%s-%s' % (id_robot, id_agent, st + 1),
                     extra_dep=all_learned)
                    
                    all_tranches.append(tranche)
                
                all_servo = comp(checkpoint, 'all servo',
                                job_id='servo-%s-%s' % (id_robot, id_agent),
                                extra_dep=all_tranches)
                
                summaries = comp(servo_stats_summaries, data_central, id_agent, id_robot,
                                 job_id='servo-summary-%s-%s' % (id_robot, id_agent),
                                 extra_dep=all_servo)    
                
                comp(servo_stats_report, data_central, id_agent, id_robot, summaries,
                     job_id='servo-report-%s-%s' % (id_robot, id_agent))


                # todo: temporary videos
                for i in range(num_ep_serv_v):
                    comp(create_video,
                         data_central=data_central,
                         id_episode=episode_id_servoing(i),
                         id_agent=id_agent,
                         id_robot=id_robot,
                         model='boot_log2movie_servo',
                         zoom=2,
                         job_id='video-serv-%s-%s-%s' % 
                            (id_robot, id_agent, episode_id_servoing(i)),
                         extra_dep=all_servo)
            
            has_predictor = agent_has_predictor(data_central, id_agent)  
            if not has_predictor:
                logger.debug('Agent %s does not support predicting.' % id_agent)
             
            if has_predictor:
                # FIXME: here we are using *all* streams 
                comp(task_predict, data_central=data_central,
                     id_agent=id_agent, id_robot=id_robot,
                     interval_print=5,
                     job_id='predict-%s-%s' % (id_robot, id_agent),
                     extra_dep=all_learned)

            
def add_exploration_videos(data_central, id_robot, id_agent,
                            episode2tranche, num_ep_expl_v):
    
    from compmake import comp 
    
    for i in range(num_ep_expl_v):
        id_episode = episode_id_exploration(i)
        comp(create_video,
             data_central=data_central,
             id_episode=id_episode,
             id_robot=id_robot,
             id_agent=id_agent,
             zoom=0,
             job_id='video-expl-%s-%s' % (id_robot, id_episode),
             extra_dep=episode2tranche[id_episode])
        comp(create_video,
             zoom=1.5,
             data_central=data_central,
             id_episode=id_episode,
             id_robot=id_robot,
             id_agent=id_agent,
             job_id='video-expl-%s-%s-zoom' % (id_robot, id_episode),
             extra_dep=episode2tranche[id_episode])
    
#    all_episodes = [episode_id_exploration(i) 
#                     for i in range(num_ep_expl_v)]
#    extra_dep = [episode2tranche[id_episode] for id_episode in all_episodes]
#        
#    comp(create_video_all_episodes,
#             data_central=data_central,
#             id_robot=id_robot,
#             episodes=all_episodes,
#             zoom=0,
#             job_id='video-expl-%s-epall' % (id_robot),
#             extra_dep=extra_dep)
#      
#    comp(create_video_all_episodes,
#             data_central=data_central,
#             id_robot=id_robot,
#             episodes=all_episodes,
#             zoom=2,
#             job_id='video-expl-%s-epall-zoom' % (id_robot),
#             extra_dep=extra_dep)
  
def agent_has_predictor(data_central, id_agent):
    agent = data_central.get_bo_config().agents.instance(id_agent)
    return hasattr(agent, 'get_predictor')
    
def agent_has_servo(data_central, id_agent):
    agent = data_central.get_bo_config().agents.instance(id_agent)
    return hasattr(agent, 'get_servo')
            
def create_video(data_central, id_episode, id_robot, id_agent, model='boot_log2movie', zoom=0):
    ds = data_central.get_dir_structure()
    basename = ds.get_video_basename(id_robot, id_agent, id_episode)
    if zoom:
        basename += '-z%.1f' % zoom

    logdir = ds.get_simulated_logs_dir()
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_episode=id_episode,
                zoom=zoom,
                output=basename)
    print('Creating video %s using model %r.' % (basename, model))
    pg(model, config=config)


def create_video_all_episodes(data_central, id_robot, id_agent, episodes=None, zoom=0):
    ''' Creates a video containing all episodes. '''
    # TODO: implement episodes
    ds = data_central.get_dir_structure()
    basename = ds.get_video_basename(id_robot, id_agent=id_agent, id_episode='all')
    if zoom:
        basename += '-z%.1f' % zoom
    
    logdir = ds.get_simulated_logs_dir()
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_episode='',
                zoom=zoom,
                output=basename)
    pg('boot_log2movie', config=config)

def checkpoint(msg):
    ''' Dummy function for job. '''
    print(msg)
