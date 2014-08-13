from blocks import (
    CheckSequence, CheckSequenceTN, Finished, NeedInput, NotReady, check_timed_named, 
    series)
from bootstrapping_olympics import ExplorableRobot, ExploringAgent, logger
from contracts import contract
from contracts.utils import check_isinstance, raise_wrapped
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
    
    
    
    agent_sys = agent.get_explorer()
    agent_sys.set_name_for_log('agent_sys')
    
    robot_sys = series(CheckSequenceTN(), robot_sys, CheckSequenceTN())
    robot_sys.set_names(['check-before-R', 'R', 'check-after-R'])
    agent_sys =  series(CheckSequenceTN(), agent_sys, CheckSequenceTN())
    robot_sys.set_names(['check-before-A', 'A', 'check-after-A'])
    
    
    robot_sys.reset()  # = new_episode
    
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
    last_u_timestamp = None
    last_y_timestamp = None
    num_default_given = 0
    while counter < max_observations:
        #print('counter: %d' % counter)
        while True: # loop until NeedInput
            
            try:
                x = robot_sys.get(block=True)
                
                check_timed_named(x)
                t, (signal, v) = x
                
                if signal == 'observations':
                    obs = v
                    if check_valid_values:
                        obs_spec.check_valid_value(obs)
                        
                    if last_y_timestamp == t:
                        msg = 'robot gave repeated observations %s' % t
                        raise ValueError(msg)
            
                    yield t, ('observations', obs)        
                    last_y_timestamp = t
                    #print('-> agent  t = %.5f' % t)
                    agent_sys.put((t, ('observations', obs)), block=True)
            
                else:
                    other_pass_through = ['robot_pose']
                    if signal in other_pass_through:
                        # yield but do not send agent
                        yield (t, (signal, v))
                    else:
                        msg = 'Invalid signal %r from robot.' % (signal)
                        raise ValueError(msg)
                
                if t0 is None:
                    t0 = t
            
            except NeedInput as e:
                if counter == 0:
                    msg = 'Robot must provide first observations'
                    msg += ' so we do not need to make up timestamp.'
                    raise_wrapped(ValueError, e, msg, counter=counter)
                break
            except NotReady:
                assert False, 'Hey, cannot obtain NotReady when block is True'
            except Finished:
                logger.info('Episode ended at %s due to obs.episode_end.'
                             % counter)
                break
            counter += 1
        
        while True:
            try:
                x = agent_sys.get(block=True)
                check_timed_named(x)
                tu, (signal, commands) = x
                #print('<- agent: %.5f %s' % (tu, signal))
                if signal != 'commands':
                    msg = 'Invalid signal %r from agent.' % signal
                    raise ValueError(msg)
                
                cmd_spec.check_valid_value(commands)
                
                if last_y_timestamp is not None and tu < last_y_timestamp:
                    msg = ('Agent command with timestamp before observations: '
                           ' %.5f < %.5f' % (tu, last_y_timestamp))
                    
                    msg += '\nI will just ignore and see if it has something better.'
                    logger.error(msg)
                    tu = last_y_timestamp
                    #continue
                    #raise ValueError(msg)
                            
            except NeedInput:
                if counter > 3:
                    msg = 'Agent raises NeedInput after %s steps.' % counter
                    raise Exception(msg)
                else:
                    logger.warn('after %d steps, using default commands because Agent not ready.' % counter)
                    logger.warn('using timestmpa %s' % last_y_timestamp)
                    
                tu = last_y_timestamp
                commands = cmd_spec.get_default_value()
                num_default_given += 1
                
            
            if last_u_timestamp is not None:
                assert tu >= last_u_timestamp
                    
            if tu == last_u_timestamp:
                msg = ('At counter = %d, repeated timestamp for commands %s' % (counter, t))
                if num_default_given:
                    logger.error(msg)
                    logger.error('Just ignoring.')
                    logger.error('This is because we needed to put n = %d default commands.' % num_default_given)
                else:
                    raise Exception(msg)
            
            yield tu, ('commands', commands)
            #print('-> robot %.5f %s' % (tu, 'commands'))
            robot_sys.put((tu, ('commands', commands)), block=True)
            last_u_timestamp = tu
        
            break
        
        # We want to end after the agent gave a command so that one 
        # observation is ready            
        time_from_episode_start = t - t0
        if time_from_episode_start > max_time:
            logger.info('Episode ended at %s for time limit %s > %s ' % 
                         (counter, time_from_episode_start, max_time))
            break

        counter += 1 

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

 
