from .load_agent_state_imp import load_agent_state
from blocks import (CheckSequence, IteratorSource, Source, bb_pump_block_yields, 
    check_timed_named, series)
from boot_manager import DataCentral, LearningState
from bootstrapping_olympics import (LearningAgent, LearningConverged, 
    get_conftools_agents, get_conftools_robots, logger)
from contracts import check_isinstance, contract
from rawlogs import RawLog
from rawlogs.library import RemoveSignals
from rawlogs_hdflog import HDFRawLog
from streamels import BootSpec
import warnings


__all__ = [
           'learn_log',
    'learn_log_source',
]
  
@contract(episodes='None|list(str)',
          parallel_hint='None|tuple(int,int)',
          returns='tuple(*,*)')
def learn_log(data_central, id_agent, id_robot,
              reset=False,
              episodes=None,
              save_state=True,
              parallel_hint=None):
    ''' If episodes is not None, it is a list of episodes id to learn. '''
     
     
    warnings.warn('add publish_interval plugins')
     
    agent0, state0 = load_agent_state(data_central,
                                      id_agent=id_agent,
                                      id_robot=id_robot,
                                      reset_state=reset)
     
    check_isinstance(agent0, LearningAgent)
     
    if parallel_hint is not None:
        #logger.info('setting parallel hint: %r' % str(parallel_hint))
        agent0.parallel_process_hint(*parallel_hint)
     
    boot_spec = get_robot_boot_spec(data_central, id_robot)
    #     source = BootStreamAsSource(stream, set(id_episode))
    
    if episodes is None:
        index = data_central.get_log_index()
        episodes = index.get_episodes_for_robot(id_robot=id_robot)
    agent, state = learn_log_base(data_central=data_central,
                                  id_agent=id_agent,
                                  agent_state=(agent0, state0),
                                  id_robot=id_robot, episodes=episodes,
                                  boot_spec=boot_spec)
 
    # Saving agent state
    if save_state:
        #logger.debug('Saving state (end of streams)')
        state.agent_state = agent.get_state()
        db = data_central.get_agent_state_db()
        db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
           
    return agent, state

@contract(data_central=DataCentral, id_robot=str)
def get_robot_boot_spec(data_central, id_robot):
    config= get_conftools_robots()
    
    has_instance = id_robot in config 
       
    if has_instance:
        robot = config.instance(id_robot)
        boot_spec = robot.get_spec()
        return boot_spec
    else:
        index = data_central.get_log_index()
        has_log = index.has_streams_for_robot(id_robot)
        
        if has_log:
            boot_spec = index.get_robot_spec(id_robot)
            return boot_spec
        else:
            raise ValueError(id_robot)
            
@contract(data_central=DataCentral, id_agent=str,
          agent_state='tuple(*,*)',
          id_robot=str,
          episodes='seq(str)',
          boot_spec=BootSpec)
def learn_log_base(data_central, id_agent,
                   agent_state, id_robot,
                   episodes, boot_spec):
    for id_episode in episodes:
        rawlog = rawlog_from_episode(data_central, id_robot, id_episode)
        agent_state = learn_rawlog(rawlog, id_agent, boot_spec, id_episode)
    return agent_state



@contract(returns=RawLog, data_central=DataCentral, id_robot='str', id_episode=str)
def rawlog_from_episode(data_central, id_robot, id_episode):
    index = data_central.get_log_index()
    nstreams = 0
    for stream in index.get_streams_for_robot(id_robot):
        nstreams += 1
        eps = stream.get_id_episodes()
        if id_episode in eps:
            filename = stream.get_filename()
            logger.error('Need to limit to this episode.')
            log = HDFRawLog(filename)
            # the "boot_info" signal has timestamp 0 which confuses the rest
            log2 = RemoveSignals(log, ['boot_info'])
            return log2
    else:
        msg = 'Could not find episode %r in %d streams.' %(id_episode, nstreams)
        raise ValueError(msg)


@contract(rawlog=RawLog, id_agent='str', boot_spec=BootSpec,
          id_episode=str)
def learn_rawlog(rawlog, id_agent, boot_spec, id_episode):
    agent = get_conftools_agents().instance(id_agent)  
    agent.init(boot_spec)
    state = LearningState(id_robot='<unnamed>', id_agent=id_agent)
    agent_state0 = agent, state
    source = source_from_rawlog(rawlog)
    agent_state = learn_log_source(agent_state0, source, id_episode)
    return agent_state
    
@contract(rawlog=RawLog, returns=Source)
def source_from_rawlog(rawlog):
    return RawLogAsSource(rawlog, topics=['observations', 'commands'])     

class RawLogAsSource(IteratorSource):
    def __init__(self, rawlog, topics, start=None, stop=None):
        self.rawlog = rawlog
        self.topics = topics
        self.start = start
        self.stop = stop
        self.rawlog = rawlog
        
    def get_iterator(self):
        it = self.rawlog.read(topics=self.topics,
                               start=self.start,
                               stop=self.stop)
        return it
    
@contract(source=Source,
          id_episode=str,
          returns='tuple(*,*)')    
def learn_log_source(agent_state, source, id_episode):
    """
        id_episode: just a string added to the learning state
        Useful to check that we are not double learning data.
    """
    agent, state = agent_state
    
    check_isinstance(agent, LearningAgent) 
 
#     for i, (stream, to_learn) in enumerate(remain):
#         compmake_progress('stream', (i, len(remain)), stream)
#  
#         for j, id_episode in enumerate(to_learn):
#             compmake_progress('episode', (j, len(to_learn)), id_episode)
#             
    # Let's reset the learner every time we start a new episode
    learner = agent.get_learner_as_sink()
    learner.reset()

#     source = BootStreamAsSource(stream, set(id_episode))
    source = series(source, CheckSequence())
    source.reset()
    
    try:
        for obs in bb_pump_block_yields(source, learner):
            check_timed_named(obs)
            
            # TODO: get length of episode
            # compmake_progress('t', (obs[0], None))
            state.num_observations += 1 
             
    except LearningConverged as e:
        print('Obtained learning converged: %s' % e)

    state.id_episodes.add(id_episode) 

    return agent, state 




# 
# class ProgressSingle(object):
#     # total > target = done + todo
#     def __init__(self, zero=0):
#         self.done = zero
#         self.todo = zero
#         self.target = zero 
#         self.total = zero
#         
#     def summary(self, units, fmt='%s'):
#         s_done = fmt % self.done
#         s_todo = fmt % self.todo
#         s_target = fmt % self.target
#         s_total = fmt % self.total
#         s = ""
#         s += "target: %s %s of %s total" % (s_target, units, s_total)
#         s += "; done: %s %s, todo: %s" % (s_done, units, s_todo)  
#         return s
#     
# class Progress(object):
#     def __init__(self):
#         self.len = ProgressSingle(0.0)
#         self.obs = ProgressSingle()
#         self.eps = ProgressSingle()
#         self.streams = ProgressSingle()
#         
#     def summary(self):
#         s = ""
#         s += "      length: %s\n" % self.len.summary('sec', '%8.1f') 
#         s += "observations: %s\n" % self.obs.summary('obs', '%8d')
#         s += "    episodes: %s" % self.eps.summary('eps', '%8d')
#         return s
# 
# 
# @contract(returns='tuple(list(tuple(*,*)),*)')
# def find_episodes_to_learn(log_index, id_robot, episodes_to_learn=None, episodes_learned=None):
#     """ 
#         Returns tuples of the kind:
#         
#             [(id_stream,  episodes_to_learn)], progress
#     
#     """
#     logger.info('Finding episodes for %s' % id_robot)
#     logger.info('To learn: %s' % episodes_to_learn)
#     logger.info('Learned:  %s' % episodes_learned)
#     
#     if not log_index.has_streams_for_robot(id_robot):
#         msg = ('No log for robot %r found.\nI have logs for: %s.'
#                % (id_robot, ", ".join(log_index.robots2streams.keys())))
#         dirnames = log_index.get_indexed_dirs()
#         msg += '\nIndexed directories:\n'
#         msg += '\n'.join('- %s' % d for d in dirnames)
#         raise ValueError(msg)
# 
#     if episodes_learned is None:
#         episodes_learned = set()
#         
#     if episodes_to_learn is None:
#         episodes_to_learn = log_index.get_episodes_for_robot(id_robot)
#  
#     p = Progress()
#  
#     episodes_found = set()
#     
#     remain = []
#     for stream in log_index.get_streams_for_robot(id_robot):
#         to_learn_in_stream = []
#         for episode_summary in stream.get_episodes():
#             id_episode = episode_summary.get_id_episode()
#             episodes_found.add(id_episode)
#             
#             ep_length = episode_summary.get_length()
#             ep_nobs = episode_summary.get_num_observations()
#             
#             p.len.total += ep_length
#             p.eps.total += 1
#             p.obs.total += ep_nobs
#         
#             target = id_episode in episodes_to_learn
#             done = id_episode in episodes_learned
#             
#             if target:
#                 p.len.target += ep_length
#                 p.eps.target += 1
#                 p.obs.target += ep_nobs
#             
#                 if done:
#                     p.len.done += ep_length
#                     p.eps.done += 1
#                     p.obs.done += ep_nobs
#                 else:                
#                     p.len.todo += ep_length
#                     p.eps.todo += 1
#                     p.obs.todo += ep_nobs
#                     
#             if target and not done:
#                 to_learn_in_stream.append(id_episode)
#         
#         if to_learn_in_stream:
#             remain.append((stream, to_learn_in_stream))
# 
#             
#     # check to see that we found all episodes we needed to learn
#     for e in episodes_to_learn:
#         if not e in episodes_found:
#             msg = 'I could not find the episode %r.' % e
#             raise ValueError(msg)
# 
#     return remain, p

