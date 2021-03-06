from .load_agent_state import load_agent_state
from bootstrapping_olympics import PassiveAgentInterface, logger
from contracts import contract
import warnings


__all__ = ['learn_log', 'learn_log_base']

@contract(episodes='None|list(str)',
          parallel_hint='None|tuple(int,int)',
          returns='tuple(*,*)')
def learn_log(data_central, id_agent, id_robot,
              reset=False,
              publish_interval=None,
              publish_once=False,
              interval_save=None,
              interval_print=None,
              episodes=None,
              save_state=True,
              parallel_hint=None,
              live_plugins=[]):
    ''' If episodes is not None, it is a list of episodes id to learn. '''
    from bootstrapping_olympics.library.live_plugins import CompmakeProgress
    from bootstrapping_olympics.library.live_plugins import PrintStatus
    
    logger.info('id_agent: %r\nepisodes:\n%r' % (id_agent, episodes))

    bo_config = data_central.get_bo_config()

    live_plugins = [bo_config.live_plugins.instance(x)
                    for x in live_plugins]
    
    live_plugins.append(CompmakeProgress())
    live_plugins.append(PrintStatus(interval_print))
    # if publish_interval is not None:
    warnings.warn('add publish_interval plugins')
#    if publish_interval is not None:
#    if 0 == state.num_observations % publish_interval:
#        phase = 'learn-active'
#        filename = ds.get_report_filename(id_agent=id_agent, id_robot=id_robot,
#                                          id_state=state.id_state, phase=phase)
# 
#        publish_agent_output(data_central, state, agent, '%05d' % state.num_observations,
#                             filename)


    
    agent0, state0 = load_agent_state(data_central, id_agent=id_agent,
                                    id_robot=id_robot, reset_state=reset)
    
    if parallel_hint is not None:
        logger.info('setting parallel hint: %r' % str(parallel_hint))
        agent0.parallel_process_hint(*parallel_hint)
    
    agent, state = learn_log_base(data_central, id_agent, (agent0, state0),
                                    id_robot, episodes,
                                    live_plugins=[])

    # Saving agent state
    if save_state:
        logger.debug('Saving state (end of streams)')
        state.agent_state = agent.get_state()
        # ds = data_central.get_dir_structure()
        db = data_central.get_agent_state_db()
        db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
          
    return agent, state

@contract(episodes='None|list(str)',
          returns='tuple(*,*)')    
def learn_log_base(data_central, id_agent, agent_state, id_robot, episodes,
                   live_plugins=[], ignore_learned=False):
    log_index = data_central.get_log_index()
    agent, state = agent_state
    
    if ignore_learned:
        episodes_learned = set()
    else:
        episodes_learned = state.id_episodes
        
    remain, progress = find_episodes_to_learn(log_index, id_robot,
                                              episodes_to_learn=episodes,
                                              episodes_learned=episodes_learned)

    logger.info('Progress for %r %r:\n%s' % (id_robot, id_agent, progress.summary()))
 
    # Initialize plugins
    init = dict(data_central=data_central, id_agent=id_agent, id_robot=id_robot)
    for plugin in live_plugins:
        plugin.init(init)

    for stream, to_learn in remain:
        
        # plugins
        for plugin in live_plugins:
            plugin.starting_stream(stream)
        
        for obs in stream.read(only_episodes=to_learn):
            state.num_observations += 1
            progress.obs.done += 1

            try:
                agent.process_observations(obs)
            except PassiveAgentInterface.LearningConverged as e:
                logger.info('Learning converged: %s' % e)
                break
            
            # Update plugins
            up = dict(agent=agent, robot=None, obs=obs, progress=progress,
                      state=state, stream=stream)
            for plugin in live_plugins:
                plugin.update(up)

        state.id_episodes.update(to_learn)
        progress.eps.done += len(to_learn)
        
    return agent, state 

# if save_state and tracker_save.its_time():
#     logger.debug('Saving state (periodic)')
#     # note: episodes not updated
#     state.agent_state = agent.get_state()
#     db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
#             if publish_interval is not None:
#                 if 0 == state.num_observations % publish_interval:
#                     phase = 'learn-active'
#                     filename = ds.get_report_filename(id_agent=id_agent, id_robot=id_robot,
#                                                       id_state=state.id_state, phase=phase)
# 
#                     publish_agent_output(data_central, state, agent, '%05d' % state.num_observations,
#                                          filename)




class ProgressSingle(object):
    # total > target = done + todo
    def __init__(self, zero=0):
        self.done = zero
        self.todo = zero
        self.target = zero 
        self.total = zero
        
    def summary(self, units, fmt='%s'):
        s_done = fmt % self.done
        s_todo = fmt % self.todo
        s_target = fmt % self.target
        s_total = fmt % self.total
        s = ""
        s += "target: %s %s of %s total" % (s_target, units, s_total)
        s += "; done: %s %s, todo: %s" % (s_done, units, s_todo)  
        return s
    
class Progress(object):
    def __init__(self):
        self.len = ProgressSingle(0.0)
        self.obs = ProgressSingle()
        self.eps = ProgressSingle()
        self.streams = ProgressSingle()
        
    def summary(self):
        s = ""
        s += "      length: %s\n" % self.len.summary('sec', '%8.1f') 
        s += "observations: %s\n" % self.obs.summary('obs', '%8d')
        s += "    episodes: %s" % self.eps.summary('eps', '%8d')
        return s

def find_episodes_to_learn(log_index, id_robot, episodes_to_learn=None, episodes_learned=None):
    """ 
        Returns tuples of the kind:
        
            (id_stream,  episodes_to_learn), progress
    
    """
    logger.info('Finding episodes for %s' % id_robot)
    logger.info('To learn: %s' % episodes_to_learn)
    logger.info('Learned:  %s' % episodes_learned)
    
    if not log_index.has_streams_for_robot(id_robot):
        msg = ('No log for robot %r found.\nI have logs for: %s.'
               % (id_robot, ", ".join(log_index.robots2streams.keys())))
        dirnames = log_index.get_indexed_dirs()
        msg += '\nIndexed directories:\n'
        msg += '\n'.join('- %s' % d for d in dirnames)
        raise ValueError(msg)


    if episodes_learned is None:
        episodes_learned = set()
        
    if episodes_to_learn is None:
        episodes_to_learn = log_index.get_episodes_for_robot(id_robot)
 
    p = Progress()
 
    episodes_found = set()
    
    remain = []
    for stream in log_index.get_streams_for_robot(id_robot):
        to_learn_in_stream = []
        for episode_summary in stream.get_episodes():
            id_episode = episode_summary.get_id_episode()
            episodes_found.add(id_episode)
            
            ep_length = episode_summary.get_length()
            ep_nobs = episode_summary.get_num_observations()
            
            p.len.total += ep_length
            p.eps.total += 1
            p.obs.total += ep_nobs
        
            target = id_episode in episodes_to_learn
            done = id_episode in episodes_learned
            
            if target:
                p.len.target += ep_length
                p.eps.target += 1
                p.obs.target += ep_nobs
            
                if done:
                    p.len.done += ep_length
                    p.eps.done += 1
                    p.obs.done += ep_nobs
                else:                
                    p.len.todo += ep_length
                    p.eps.todo += 1
                    p.obs.todo += ep_nobs
                    
            if target and not done:
                to_learn_in_stream.append(id_episode)
        
        if to_learn_in_stream:
            remain.append((stream, to_learn_in_stream))

            
    # check to see that we found all episodes we needed to learn
    for e in episodes_to_learn:
        if not e in episodes_found:
            msg = 'I could not find the episode %r.' % e
            raise ValueError(msg)

    return remain, p

