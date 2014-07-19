from blocks import NeedInput, NotReady
from blocks.composition import series
from blocks import Finished
from blocks.library import CheckSequence
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics import (ExplorableRobot, ExploringAgent, logger)
from contracts import contract
from contracts.utils import check_isinstance
import time
import warnings

__all__ = ['run_simulation']


@contract(id_robot='str', id_agent='str',
          robot=ExplorableRobot, agent=ExploringAgent, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, id_agent, agent, max_observations,
                   max_time,
                   check_valid_values=True):
    ''' 
        Runs one episode of the simulation until it ends. 
        
        The agent should already been init()ed. 
        
        Yields:
            (timestamp, ('observations', obs))
        and
             (timestamp, ('commands', cmd))
    '''

    check_isinstance(agent, ExploringAgent)
    check_isinstance(robot, ExplorableRobot)

    warnings.warn('we are not honoring id_episode')


    logger.info('run_simulation(max_time=%s; max_observations=%s)'
                % (max_time, max_observations))

    
    robot_as_sys = robot.get_active_stream()
    
    
    logger.info('max_observations: %s' % max_observations)
    logger.info('max_time: %s' % max_time)

    
    if True:
        robot_sys = series(robot_as_sys,  # CheckBootSpec(robot.get_spec()),
                            CheckSequence())

        robot_sys.set_names(['robot_sys', 'CheckSeq'])
        robot_sys.set_name_for_log('robot_sys')
    else:
        robot_sys = robot_as_sys
    robot_sys.reset()  # = new_episode
    
    agent_sys = agent.get_explorer()
    agent_sys.set_name_for_log('agent_sys')
    agent_sys.reset()
    
    for x in run_simulation_systems(robot_sys=robot_sys, 
                                    agent_sys=agent_sys, 
                                    boot_spec=robot.get_spec(), 
                                    max_observations=max_observations, 
                                    max_time=max_time,
                                    check_valid_values=check_valid_values):
        yield x

def run_simulation_systems(robot_sys, agent_sys, boot_spec, max_observations, max_time,
                           check_valid_values):
    cmd_spec = boot_spec.get_commands()
    obs_spec = boot_spec.get_observations()

    counter = 0
    t0 = None
    while counter < max_observations:
        try:
            x = robot_sys.get(block=True)
            check_timed_named(x)
            t, (signal, v) = x
            check_isinstance(t, float)
            if signal == 'observations':
                obs = v
                if check_valid_values:
                    obs_spec.check_valid_value(obs)
            else:
                msg = 'Invalid signal %r from robot.' % (signal)
                raise ValueError(msg)
            if t0 is None:
                t0 = t
        
        except NeedInput:
            if counter == 0:
                logger.info('Adding input to robot')
                cmd0 = cmd_spec.get_default_value()
                robot_sys.put((time.time(), ('commands', cmd0)))
                continue
            else:
                raise
        except NotReady:
            assert False, 'Hey, cannot obtain NotReady when block is True'
        except Finished:
            logger.info('Episode ended at %s due to obs.episode_end.'
                         % counter)
            break
            
        yield t, ('observations', obs)
            
        time_from_episode_start = t - t0
        if time_from_episode_start > max_time:
            logger.info('Episode ended at %s for time limit %s > %s ' % 
                         (counter, time_from_episode_start, max_time))
            break

        # logger.info('putting into agent')
        agent_sys.put((t, ('observations', obs)), block=True)

        # logger.info('getting commands')
        try:
            x = agent_sys.get(block=True)
            check_timed_named(x)
            t, (signal, commands) = x
            if signal != 'commands':
                msg = 'Invalid signal %r from robot.' % signal
                raise ValueError(msg)
            cmd_spec.check_valid_value(commands)
        except NeedInput:
            if counter > 3:
                msg = 'Agent raises NeedInput after %s steps.' % counter
                raise Exception(msg)
            else:
                logger.warn('after %d steps, using default commands because Agent not ready.' % counter)
            commands = cmd_spec.get_default_value()

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        # logger.info('putting into robots')
        tu = t
        yield tu, ('commands', commands)
        robot_sys.put((tu, ('commands', commands)), block=True)

        counter += 1

 
