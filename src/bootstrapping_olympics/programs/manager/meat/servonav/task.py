from . import get_grid
from ..servo import BookkeepingServo, get_vsim_from_robot
from bootstrapping_olympics import LogsFormat, RobotInterface, logger
from bootstrapping_olympics.programs.manager.meat import load_agent_state
from bootstrapping_olympics.utils import InAWhile, unique_timestamp_string
from contracts import contract
import numpy as np


__all__ = ['task_servonav']

@contract(interval_print='None|>=0')
def task_servonav(data_central, id_agent, id_robot,
               max_episode_len,
               num_episodes,
               fail_if_not_working,
               id_episodes=None,  # if None, just use the ID given by the world
               cumulative=False,
               interval_print=None,
               interval_write=10,  # write every 10 frames
               num_episodes_with_robot_state=0,
                resolution=1):
    ''' Returns the list of the episodes IDs simulated. '''

    # Reseed the generator (otherwise multiprocessing will use the same)
    np.random.seed()

    if id_episodes is not None:
        if len(id_episodes) != num_episodes:
            raise ValueError('Expected correct number of IDs.')

    # Instance robot object
    robot = data_central.get_bo_config().robots.instance(id_robot)

    # TODO: check that this is a Vehicles simulation

    boot_spec = robot.get_spec()

    # Instance agent object    
    agent, _ = load_agent_state(data_central, id_agent, id_robot,
                                reset_state=False,
                                raise_if_no_state=True)
    # TODO: check servo
    servo_agent = agent.get_servo()
    id_agent_servo = '%s_servo' % id_agent

    ds = data_central.get_dir_structure()
    id_stream = '%s_%s_%s_servonav' % (id_robot, id_agent,
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

    if not bk.another_episode_todo():
        return

    with logs_format.write_stream(filename=filename,
                                  id_stream=id_stream,
                                  boot_spec=boot_spec) as writer:
        counter = 0
        while bk.another_episode_todo():
            episode = robot.new_episode()

            if id_episodes is not None:
                id_episode = id_episodes.pop(0)
            else:
                id_episode = episode.id_episode

            save_robot_state = counter < num_episodes_with_robot_state

            servonav_episode(id_robot=id_robot, robot=robot,
                 id_servo_agent=id_agent_servo,
                 servo_agent=servo_agent,
                 writer=writer, id_episode=id_episode,
                 resolution=resolution,
                 max_episode_len=max_episode_len,
                 save_robot_state=save_robot_state,
                 interval_write=interval_write,
                 fail_if_not_working=fail_if_not_working,
                 max_tries=10000)

            bk.episode_done()
            counter += 1


def convert_to_yaml(locations):
    from geometry import SE3

    def convert(loc):
        loc = dict(**loc)
        loc['pose'] = SE3.to_yaml(loc['pose'])
        loc['observations'] = loc['observations'].tolist()
        return loc
    return [convert(l) for l in locations]


def servonav_episode(id_robot, robot,
                     id_servo_agent, servo_agent,
                     writer, id_episode,
                     max_episode_len, save_robot_state,
                     interval_write=1,
                     interval_print=5,
                     resolution=0.5,  # grid resolution
                     delta_t_threshold=0.2,  # when to switch
                     MIN_PATH_LENGTH=8,
                     MAX_TIME_FOR_SWITCH=20.0,
                     fail_if_not_working=False,
                     max_tries=10000):
    '''
    
        :arg:displacement: Time in seconds to displace the robot.
    '''
    from geometry import (SE2_from_SE3, translation_from_SE2,
                          angle_from_SE2, SE3)

    stats_write = InAWhile(interval_print)

    # Access the vehicleSimulation interface
    vsim = get_vsim_from_robot(robot)

    for _ in xrange(max_tries):
        # iterate until we can do this correctly
        episode = robot.new_episode()
        locations = get_grid(robot=robot, vsim=vsim, resolution=resolution)

        if len(locations) < MIN_PATH_LENGTH:
            logger.info('Path too short, trying again.')
        else:
            break

    else:
        msg = 'Could not find path in %d tries.' % max_tries
        raise Exception(msg)

    locations_yaml = convert_to_yaml(locations)

    vsim.vehicle.set_pose(locations[0]['pose'])

    current_goal = 1
    current_goal_obs = locations[current_goal]['observations']
    servo_agent.set_goal_observations(current_goal_obs)

    counter = 0
    time_last_switch = None

    num_written = 0
    for robot_observations, boot_observations in \
        run_simulation_servonav(id_robot, robot,
                           id_servo_agent, servo_agent,
                           100000, max_episode_len,
                           id_episode=id_episode,
                           id_environment=episode.id_environment,
                           raise_error_on_collision=fail_if_not_working):

        current_time = boot_observations['timestamp'].item()
        if time_last_switch is None:
            time_last_switch = current_time

        time_since_last_switch = float(current_time - time_last_switch)

        def obs_distance(obs1, obs2):
            return float(np.linalg.norm(obs1.flatten() - obs2.flatten()))

        curr_pose = robot_observations.robot_pose
        curr_obs = boot_observations['observations']
        curr_goal = locations[current_goal]['observations']
        prev_goal = locations[current_goal - 1]['observations']
        curr_err = obs_distance(curr_goal, curr_obs)
        prev_err = obs_distance(prev_goal, curr_obs)
        current_goal_pose = locations[current_goal]['pose']
        current_goal_obs = locations[current_goal]['observations']

        delta = SE2_from_SE3(SE3.multiply(SE3.inverse(curr_pose),
                                          current_goal_pose))
        delta_t = np.linalg.norm(translation_from_SE2(delta))
        delta_th = np.abs(angle_from_SE2(delta))

        if stats_write.its_time():
            msg = ('  deltaT: %.2fm  deltaTh: %.1fdeg' % 
                     (delta_t, np.rad2deg(delta_th)))
            logger.debug(msg)

        # If at the final goal, go closer
        is_final_goal = current_goal == len(locations) - 1
        if is_final_goal:
            delta_t_threshold *= 0.3

        # TODO: should we care also about delta_th?
        time_to_switch = (delta_t < delta_t_threshold) or \
            (time_since_last_switch > MAX_TIME_FOR_SWITCH)
        # does not work: curr_err < SWITCH_THRESHOLD * prev_err:

        if time_to_switch:
            current_goal += 1
            logger.info('Switched to goal %d.' % current_goal)

            time_last_switch = current_time
            if current_goal >= len(locations):
                # finished
                logger.info('Finished :-)')
                break

        threshold_lost_m = 3
        if delta_t > threshold_lost_m:
            msg = 'Breaking because too far away.'
            if not(fail_if_not_working):
                logger.error(msg)
                break
            else:
                raise Exception(msg)

        servo_agent.set_goal_observations(current_goal_obs)

        extra = {}
        extra['servoing_base'] = dict(goal=curr_goal.tolist(),
                                      current=curr_obs.tolist())

        extra['servoing_poses'] = dict(goal=SE3.to_yaml(current_goal_pose),
                                       current=SE3.to_yaml(curr_pose))

        extra['servonav'] = dict(poseK=SE3.to_yaml(curr_pose),
                        obsK=boot_observations['observations'].tolist(),
                        pose1=SE3.to_yaml(current_goal_pose),
                        locations=locations_yaml,
                        current_goal=current_goal,
                        curr_err=curr_err,
                        prev_err=prev_err,
                        time_last_switch=time_last_switch,
                        time_since_last_switch=time_since_last_switch
                        )

        if counter % interval_write == 0:
            if save_robot_state:
                extra['robot_state'] = robot.get_state()

            writer.push_observations(observations=boot_observations,
                                     extra=extra)
            num_written += 1
        counter += 1

    if num_written == 0:
        msg = ('This log was too short to be written (%d observations)'
               % counter)
        raise Exception(msg)


@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, max_observations='>=1',
          max_time='>0')
def run_simulation_servonav(id_robot, robot, id_agent, agent,
                            max_observations, max_time,
                            id_episode, id_environment,
                            check_valid_values=True,
                            raise_error_on_collision=True):
    ''' 
        Runs an episode of the simulation. The agent should already been
        init()ed. 
        
        yields robot.get_observations(), boot_observatoions
    '''

    keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)

    obs_spec = robot.get_spec().get_observations()
    cmd_spec = robot.get_spec().get_commands()

    counter = 0
    while True:
        obs = robot.get_observations()

        if check_valid_values:
            obs_spec.check_valid_value(obs.observations)

#        print('run_simulation_servonav: obs.obs %s' %
#              (str(obs.observations.shape)))
        observations = keeper.push(timestamp=obs.timestamp,
                                   observations=obs.observations,
                                   commands=obs.commands,
                                   commands_source=obs.commands_source,
                                   id_episode=id_episode,
                                   id_world=id_environment)
#        print('then: obs.obs %s' %
#              (str(observations['observations'].shape)))

        episode_end = obs.episode_end

        yield obs, observations

        now = 'step %s' % counter

        if counter >= max_observations:
            logger.info('Finished at %s because %s >= %s' % 
                        (now, counter, max_observations))
            break

        if observations['time_from_episode_start'] > max_time:
            logger.info('Finished at %s because of max_time: %s > %s' % 
                        (now, observations['time_from_episode_start'],
                         max_time))
            break

        if episode_end:  # Fishy
            msg = 'Finished at %s because of robot driver.' % now

            if raise_error_on_collision:
                raise Exception(msg)
            else:
                logger.info(msg)
                break

        agent.process_observations(observations)
        commands = agent.choose_commands()

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        robot.set_commands(commands, id_agent)

        counter += 1

