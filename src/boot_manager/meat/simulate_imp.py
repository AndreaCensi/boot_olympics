from .m_run_simulation import run_simulation
from .servo.servo import WrapSeparateEpisodes
from blocks import check_timed_named
from boot_manager import DataCentral, LogsFormat, logger
from bootstrapping_olympics import get_conftools_agents, get_conftools_robots
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
      
    with logs_format.write_stream_and_check(filename=filename,
                                              id_stream=id_stream,
                                              boot_spec=boot_spec,
                                              id_agent=id_agent,
                                              id_robot=id_robot) as writer:
        writer = WrapSeparateEpisodes(writer)
        writer.reset()

        for id_episode in id_episodes:
            logger.info('Simulating episode %s' % id_episode)
             
            count = 0
            writer.new_episode()
            for x in run_simulation(id_robot, robot, id_agent,
                                    agent, 100000, max_episode_len):
                count += 1
                check_timed_named(x)
                timestamp, (signal, value) = x
 
                log_signals = ['observations', 'commands', 'robot_pose']
                            
                if signal in log_signals:
                    writer.put((timestamp, (signal, value)))
                else:
                    msg = 'Unknown signal %r.' % signal
                    raise ValueError(msg)

                if signal == 'observations': 
                    # Put 'id_episode' after each observations
                    writer.put((timestamp, ('id_episode', id_episode)))
                
                if signal == 'observations':
                    # Put 'extra', possibly empty, after each observations
                    if write_extra:
                        extra = dict(robot_state=robot.get_state())
                    else:
                        extra = {}
                    writer.put((timestamp, ('extra', extra)))
        
            if count < 5:
                raise ValueError('Exploration only lasted %d steps.' % count)
                             
        writer.end_input()

    logger.info('Peacefully done all episodes')
 
    return id_episodes
