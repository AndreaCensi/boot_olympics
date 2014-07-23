from .m_run_simulation import run_simulation
from blocks import Sink
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics import (LogsFormat, get_conftools_agents, 
    get_conftools_robots, logger)
from bootstrapping_olympics.programs.manager.meat.data_central import (
    DataCentral)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
import numpy as np


__all__ = [
    'simulate_agent_robot',
]


@contract(data_central=DataCentral,
          id_agent='str',
          id_robot='str',
          id_episodes='list(str)',
          cumulative='bool',
          stateful='bool',
           write_extra='bool')
def simulate_agent_robot(data_central, id_agent, id_robot,
             max_episode_len,
             cumulative,
             id_episodes,
             stateful=False,
              write_extra=True):
    ''' If not cumulative, returns the list of the episodes IDs simulated,
        otherwise it returns all episodes. '''

    if id_episodes is None:
        msg = 'In the new system you have to pass explicit id_episodes'
        raise Exception(msg)
    
    # Reseed the generator (otherwise multiprocessing will use the same)
    np.random.seed()

    # Instance agent object    
    agent = get_conftools_agents().instance(id_agent) 
    # Instance robot object
    robot = get_conftools_robots().instance(id_robot) 

    boot_spec = robot.get_spec()

    # If --stateful is passed, we try to load a previous state.
    if stateful:
        db = data_central.get_agent_state_db()
        if db.has_state(id_agent=id_agent, id_robot=id_robot):
            logger.info('Using previous state.')
            db.reload_state_for_agent(id_agent=id_agent, id_robot=id_robot,
                                      agent=agent)
        else:
            logger.info('No previous state found.')
            agent.init(boot_spec)
    else:
        agent.init(boot_spec)

    ds = data_central.get_dir_structure()
    timestamp = unique_timestamp_string()
    timestamp = timestamp.replace('_', '')
    id_stream = '%s-%s-%s' % (id_robot, id_agent, timestamp)
    filename = ds.get_simlog_filename(id_robot=id_robot,
                                      id_agent=id_agent,
                                      id_stream=id_stream)
    logger.info('Creating stream %r\n in file %r' % (id_stream, filename))

    logs_format = LogsFormat.get_reader_for(filename)

    if cumulative:
        raise NotImplementedError('cumulative not implemented')
      
    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=boot_spec,
                                  id_agent=id_agent,
                                  id_robot=id_robot) as writer:
        
        assert isinstance(writer, Sink)

        writer.reset()
        last_episode_timestamp = None
        
        
        for id_episode in id_episodes:
            logger.info('Simulating episode %s' % id_episode)
            
            delta = None
            if last_episode_timestamp is None:
                delta = 0.0
                
            for x in run_simulation(id_robot, robot, id_agent,
                                    agent, 100000, max_episode_len):
                
                check_timed_named(x)
                timestamp, (signal, value) = x

                print('run_simulation gave %.4f: %s' % (timestamp, signal))
                if (delta is None) and (last_episode_timestamp is not None):
                    
                    if timestamp < last_episode_timestamp:
                        delta = last_episode_timestamp - timestamp + 10.0
                        msg = 'Due to simulation, sometimes episodes have overlapping timestamps.'
                        msg += 'I will add a delta of at least 10 seconds (delta = %.4f)' % delta
                        logger.warn(msg)
                    else:
                        delta = 0
                    
                timestamp = timestamp + delta
                                
                if signal in ['observations', 'commands']:
                    writer.put((timestamp, (signal, value)))
                else:
                    msg = 'Unknown signal %r.' % signal
                    raise ValueError(msg)

                if signal == 'observations':
                    print('putting id_episode at %s' % timestamp)
                    writer.put((timestamp, ('id_episode', id_episode)))
                
                
                if signal == 'observations':
                    if write_extra:
                        extra = dict(robot_state=robot.get_state())
                    else:
                        extra = {}
                    writer.put((timestamp, ('extra', extra)))
        
            last_episode_timestamp = timestamp
                     
        writer.end_input()
        logger.info('Peacefully done all episodes')

    return id_episodes
# 
#     if cumulative:
#         return bk.get_all_episodes()
#     else:
#         return bk.get_id_episodes()

# 
# class Bookkeeping():
#     ''' Simple class to keep track of how many we have to simulate. '''
#     def __init__(self, data_central, id_robot, num_episodes,
#                  cumulative=True, interval_print=5):
#         self.data_central = data_central
#         self.id_robot = id_robot
#         self.cumulative = cumulative
# 
#         if self.cumulative:
#             log_index = data_central.get_log_index()
#             if log_index.has_streams_for_robot(id_robot):
#                 self.done_before = log_index.get_episodes_for_robot(id_robot)
#                 self.num_episodes_done_before = len(self.done_before)
#             else:
#                 self.done_before = set()
#                 self.num_episodes_done_before = 0
#             self.num_episodes_todo = (num_episodes - 
#                                       self.num_episodes_done_before)
#             logger.info('Preparing to do %d episodes (already done %d).' % 
#                         (self.num_episodes_todo,
#                          self.num_episodes_done_before))
#         else:
#             self.num_episodes_todo = num_episodes
#             logger.info('Preparing to do %d episodes.' % 
#                         self.num_episodes_todo)
#         self.num_episodes_done = 0
#         self.num_observations = 0
#         self.num_observations_episode = 0
#         self.observations_per_episode = []
# 
#         self.interval_print = interval_print
#         self.tracker = InAWhile(interval_print)
#         self.id_episodes = set()
# 
#         try:
#             from compmake import progress
#             progress('Simulating episodes', (0, self.num_episodes_todo))
#         except ImportError:
#             pass
# 
#     def observations(self, observations):
# 
#         self.num_observations_episode += 1
#         self.num_observations += 1
#         if self.tracker.its_time():
#             msg = ('simulating %d/%d episodes obs %d (%5.1f fps)' % 
#                    (self.num_episodes_done,
#                     self.num_episodes_todo,
#                     self.num_observations, self.tracker.fps()))
#             if self.num_episodes_done > 0:
#                 msg += (' (mean obs/ep: %.1f)' % 
#                         (np.mean(self.observations_per_episode)))
#             logger.info(msg)
# 
#     def get_id_episodes(self):
#         ''' Returns the list of episodes simulated. '''
#         return natsorted(self.id_episodes)
# 
#     def get_all_episodes(self):
#         ''' Returns the list of all episodes, both the already present
#             and the simulated. '''
#         eps = []
#         eps.extend(self.id_episodes)
#         eps.extend(self.done_before)
#         return natsorted(set(eps))
# 
#     def episode_done(self, id_episode):
#         self.id_episodes.add(id_episode)
#         self.num_episodes_done += 1
#         self.observations_per_episode.append(self.num_observations_episode)
#         self.num_observations_episode = 0
# 
#         try:
#             from compmake import progress
#             progress('Simulating episodes', (self.num_episodes_done,
#                                              self.num_episodes_todo))
#         except ImportError:
#             pass
# 
#     def another_episode_todo(self):
#         return self.num_episodes_done < self.num_episodes_todo


# simulate = simulate_agent_robot
