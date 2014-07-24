
from .bookkeeping import BookkeepingServo
from blocks.library.simple.instantaneous import Instantaneous
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics import LogsFormat, get_conftools_robots, logger
from bootstrapping_olympics.interfaces.agent import ServoingAgent
from bootstrapping_olympics.programs.manager.meat import load_agent_state
from bootstrapping_olympics.programs.manager.meat.m_run_simulation import (
    run_simulation_systems)
from bootstrapping_olympics.programs.manager.meat.servonav.find_path import (
    mean_observations)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
from geometry import SE2_from_SE3, angle_from_SE2, translation_from_SE2
import numpy as np
import warnings
from blocks.interface import Sink

__all__ = ['task_servo']

@contract(interval_print='None|>=0')
def task_servo(data_central, id_agent, id_robot,
               max_episode_len,
               num_episodes,
               displacement,
               id_episodes=None,  # if None, just use the ID given by the world
               cumulative=False,
               interval_print=None,
               num_episodes_with_robot_state=0):
    ''' Returns the list of the episodes IDs simulated. '''

    # Reseed the generator (otherwise multiprocessing will use the same)
    np.random.seed()

    if id_episodes is not None:
        if len(id_episodes) != num_episodes:
            raise ValueError('Expected correct number of IDs.')

    # Instance robot object
    robot = get_conftools_robots().instance(id_robot)

    # TODO: check that this is a Vehicles simulation

    boot_spec = robot.get_spec()

    # Instance agent object    

    agent, _ = load_agent_state(data_central, id_agent, id_robot,
                             reset_state=False,
                             raise_if_no_state=True)

#     servo_agent = agent.get_servo()
#     servo_agent.init(boot_spec)

    id_agent_servo = '%s_servo' % id_agent

    ds = data_central.get_dir_structure()
    id_stream = '%s_%s_%s_servo' % (id_robot, id_agent,
                                    unique_timestamp_string())
    filename = ds.get_simlog_filename(id_robot=id_robot,
                                          id_agent=id_agent,
                                          id_stream=id_stream)
    logger.info('Creating stream %r\n in file %r' % (id_stream, filename))

    logs_format = LogsFormat.get_reader_for(filename)

    bk = BookkeepingServo(data_central=data_central,
                     id_robot=id_robot,
                     id_agent=id_agent_servo,
                     num_episodes=num_episodes,
                     cumulative=cumulative,
                     interval_print=interval_print)

    if bk.another_episode_todo():
        with logs_format.write_stream(filename=filename,
                                      id_stream=id_stream,
                                      boot_spec=boot_spec,
                                      id_agent=id_agent_servo,
                                      id_robot=id_robot) as writer:
            writer.reset()
            
            writer = WrapSeparateEpisodes(writer)
            counter = 0
            while bk.another_episode_todo():
                episode = robot.new_episode()

                if id_episodes is not None:
                    id_episode = id_episodes.pop(0)
                else:
                    id_episode = episode.id_episode

                save_robot_state = counter < num_episodes_with_robot_state

                writer.new_episode()
                servoing_episode( robot=robot,  agent=agent,
                     writer=writer, id_episode=id_episode,
                     displacement=displacement,
                     max_episode_len=max_episode_len,
                     save_robot_state=save_robot_state,
                     max_tries=10000)


                bk.episode_done()
                counter += 1
                
class WrapSeparateEpisodes(Sink):
    
    def __init__(self, sink, min_diff=60.0):
        self.sink = sink
        self.min_diff = min_diff
    
    def reset(self):
        self.sink.reset()
        self.last_episode_final_timestamp = None
        self.check_next = False
        self.last_timestamp = None
        
    def new_episode(self):
        self.last_episode_final_timestamp = self.last_timestamp
        if self.last_timestamp is not None:
            self.check_next = True
        
    def put(self, value, block=True, timeout=None):  # @UnusedVariable
        check_timed_named(value)
        timestamp, ob = value
        
        if self.check_next:
            if timestamp < self.last_episode_final_timestamp:
                self.delta = self.last_episode_final_timestamp - timestamp + self.min_diff
                msg =( 'Due to simulation, sometimes episodes have '
                       'overlapping timestamps.'
                        'I will add a delta of at least '
                        '%.4f seconds (delta = %.4f)' % (self.min_diff, self.delta))
                logger.warn(msg)
            else:
                self.delta = 0
            
        self.sink.put((timestamp + self.delta, ob), block=block)
                
        self.last_timestamp = timestamp
        
                

@contract(agent=ServoingAgent)
def servoing_episode(robot,
                     agent,
                     writer, id_episode,
                     displacement,
                     max_episode_len,
                     save_robot_state,
                     converged_dist_t_m=0.1,
                     converged_dist_th_deg=1,
                     max_tries=10000):
    '''
    
        :arg:displacement: Time in seconds to displace the robot.
    '''
    from geometry import SE3


    for ntries in xrange(max_tries):
        # iterate until we can do this correctly
        robot_sys = robot.get_active_stream()
        robot_sys.reset()
        
        rest = robot.get_spec().get_commands().get_default_value()
        t, pose0, obs0 = mean_observations(robot_sys, rest=rest, n=10)

        cmd0 = robot.get_spec().get_commands().get_random_value()
        
        ok = simulate_hold(cmd0, robot_sys, robot.get_spec(), displacement)
        if ok:
            _, pose1, _ = mean_observations(robot_sys, rest=rest, n=1)
            logger.info('Displacement after %s tries.' % ntries)
            break
    else:
        msg = 'Could not do the displacement (%d tries).' % max_tries
        raise Exception(msg)

    current_pose = None

    agent_sys = agent.get_servo_system()
    agent_sys.reset()
    print('Putting goal_observations')
    agent_sys.put((t, ('goal_observations', obs0)))
    print('put goal_observations')
    agent_sys.info('did you receive it?')
    
    simstream = run_simulation_systems(robot_sys=robot_sys, 
                                    agent_sys=agent_sys, 
                                    boot_spec=robot.get_spec(), 
                                    max_observations=100000, 
                                    max_time=max_episode_len,
                                    check_valid_values=True)
    for x in simstream: 
        check_timed_named(x)
        timestamp, (signal, value) = x

        if signal == 'robot_pose':
            current_pose = value
        elif signal == 'commands':
            writer.put((timestamp, ('commands', value)))
        elif signal == 'observations':
            observations = value
            extra = {}
    
            sensels_list = observations.tolist()
            extra['servoing_base'] = dict(goal=obs0.tolist(), current=sensels_list)
    
            has_pose = current_pose is not None
        
            if has_pose:
                # Add extra pose information
    
                extra['servoing_poses'] = dict(goal=pose_to_yaml(pose0),
                                               current=pose_to_yaml(current_pose))
    
                delta = SE2_from_SE3(SE3.multiply(SE3.inverse(current_pose),
                                                  pose0))
                dist_t_m = np.linalg.norm(translation_from_SE2(delta))
                dist_th_deg = np.abs(angle_from_SE2(delta))
    
                # TODO: make it not overlapping
                extra['servoing'] = dict(obs0=obs0.tolist(),
                                        pose0=pose_to_yaml(pose0),
                                        poseK=pose_to_yaml(current_pose),
                                        obsK=sensels_list,
                                        displacement=displacement,
                                        cmd0=cmd0.tolist(),
                                        pose1=pose_to_yaml(pose1))
    
            if save_robot_state:
                extra['robot_state'] = robot.get_state()
                
            writer.put((timestamp, ('id_episode', id_episode)))
            writer.put((timestamp, ('observations', observations)))
            writer.put((timestamp, ('extra', extra)))

        if has_pose:
            if ((dist_t_m <= converged_dist_t_m) and
                (dist_th_deg <= converged_dist_th_deg)):
                print('Converged!')
                break
        else:
            warnings.warn('TODO: write convergence criterion without pose information')
            pass

def pose_to_yaml(x):
    ''' Converts to yaml, or sets None. '''
    from geometry import SE3
    
    if x is None:
        return None
    else:
        return SE3.to_yaml(x)


@contract(returns='bool')
def simulate_hold(cmd0, robot_sys, boot_spec, displacement):
    """ Returns True if we can hold the command for the given amount of time. """
    
    class ConstantOutput(Instantaneous):
        def __init__(self, cmd0):
            self.cmd0 = cmd0
        def transform_value(self, value):
            check_timed_named(value)
            (t, (signal, _)) = value
            assert signal == 'observations'
            return (t, ('commands', self.cmd0))
            
    agent_sys = ConstantOutput(cmd0)
    agent_sys.reset()
    t0 = None
    for x in run_simulation_systems(robot_sys=robot_sys, 
                                agent_sys=agent_sys, 
                                boot_spec=boot_spec, 
                                max_observations=100000, 
                                max_time=displacement * 2,
                                check_valid_values=True):
        timestamp, (_, _) = x
        if t0 is None:
            t0 = timestamp

    length = timestamp - t0
    
    success = length > displacement * 0.99
    
    return success 
#     
#     t0 = timestamp()
#     nsteps = 0
#     while timestamp() < t0 + displacement:
#         nsteps += 1
#         source = BootOlympicsConstants.CMD_SOURCE_SERVO_DISPLACEMENT
#         robot.set_commands(cmd0, source)
#         if episode_ended():
#             logger.debug('Collision after %d steps' % ntries)
#             return False
# 
#     logger.debug('%d steps of simulation to displace by %s' % 
#                 (nsteps, displacement))
#     return True



