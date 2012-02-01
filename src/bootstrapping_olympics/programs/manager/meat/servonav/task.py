from . import get_grid, logger, contract, np
from .. import load_agent_state
from ..servo import BookkeepingServo
from bootstrapping_olympics import RobotInterface, ObsKeeper, LogsFormat
from bootstrapping_olympics.utils import isodate_with_secs


@contract(interval_print='None|>=0')
def task_servonav(data_central, id_agent, id_robot,
               max_episode_len,
               num_episodes,
               id_episodes=None, # if None, just use the ID given by the world
               cumulative=False,
               resolution=1,
               interval_print=None,
               interval_write=10, # write every 10 frames
               num_episodes_with_robot_state=0):
    ''' Returns the list of the episodes IDs simulated. '''
    from geometry import SE2_from_SE3, translation_from_SE2, angle_from_SE2, SE3

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
    id_stream = '%s-%s-%s-servonav' % (id_robot, id_agent, isodate_with_secs())
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
                 max_tries=10000)

            bk.episode_done()
            counter += 1


def convert_to_yaml(locations):
    def convert(loc):
        loc = dict(**loc)
        loc['pose'] = SE3.to_yaml(loc['pose'])
        loc['observations'] = loc['observations'].tolist()
        loc['cell'] = list(loc['cell'])
        return loc
    return [convert(l) for l in locations]


def servonav_episode(id_robot, robot,
                     id_servo_agent, servo_agent,
                     writer, id_episode,
                     max_episode_len, save_robot_state,
                     interval_write=1,
                     resolution=0.5, # grid resolution
                     max_tries=10000):
    '''
    
        :arg:displacement: Time in seconds to displace the robot.
    '''
    from geometry import SE2_from_SE3, translation_from_SE2, angle_from_SE2, SE3
    

    MIN_PATH_LENGTH = 8

    for _ in xrange(max_tries):
        # iterate until we can do this correctly
        episode = robot.new_episode()
        locations = get_grid(robot=robot,
                        world=robot.world,
                        vehicle=robot.vehicle, resolution=resolution)

        if len(locations) < MIN_PATH_LENGTH:
            print('Path too short, trying again')
        else:
            break

    else:
        msg = 'Could not do the displacement (%d tries).' % max_tries
        raise Exception(msg)

    locations_yaml = convert_to_yaml(locations)

    robot.vehicle.set_pose(locations[0]['pose'])

    current_goal = 1
    current_goal_obs = locations[current_goal]['observations']
    servo_agent.set_goal_observations(current_goal_obs)

    counter = 0
    time_last_switch = None

    MAX_TIME_FOR_SWITCH = 30.0
    #SWITCH_THRESHOLD = 0.5

    num_written = 0
    for observations in run_simulation_servonav(id_robot, robot,
                           id_servo_agent, servo_agent,
                           100000, max_episode_len,
                           id_episode=id_episode,
                           id_environment=episode.id_environment):

        current_time = observations['timestamp'].item()
        if time_last_switch is None:
            time_last_switch = current_time

        time_since_last_switch = float(current_time - time_last_switch)

        def obs_distance(obs1, obs2):
            return float(np.linalg.norm(obs1 - obs2))

        curr_pose = robot.get_observations().robot_pose
        curr_obs = observations['observations']
        curr_goal = locations[current_goal]['observations']
        prev_goal = locations[current_goal - 1]['observations']
        curr_err = obs_distance(curr_goal, curr_obs)
        prev_err = obs_distance(prev_goal, curr_obs)
        current_goal_pose = locations[current_goal]['pose']
        current_goal_obs = locations[current_goal]['observations']

        delta = SE2_from_SE3(SE3.multiply(SE3.inverse(curr_pose), current_goal_pose))
        delta_t = np.linalg.norm(translation_from_SE2(delta))
        delta_th = np.abs(angle_from_SE2(delta))

        logger.debug(#(' curr_err/prev_err: %10f ' % (curr_err / prev_err))
                    ('  deltaT: %.2fm  deltaTh: %.1fdeg' %
                     (delta_t, np.rad2deg(delta_th))))

        delta_t_threshold = 0.4
        time_to_switch = delta_t < delta_t_threshold
        # curr_err < SWITCH_THRESHOLD * prev_err:

        if time_to_switch:
            current_goal += 1
            logger.info('Switched to goal %d.' % current_goal)

            time_last_switch = current_time
            if current_goal >= len(locations):
                # finished
                logger.info('Finished :-)')
                break

        if time_since_last_switch > MAX_TIME_FOR_SWITCH:
            logger.error('breaking because too much time passed')
            break

        servo_agent.set_goal_observations(current_goal_obs)


        extra = {}
        extra['servoing_base'] = dict(goal=curr_goal.tolist(),
                                      current=curr_obs.tolist())

        extra['servonav'] = dict(poseK=SE3.to_yaml(curr_pose),
                        obsK=observations['observations'].tolist(),
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


            writer.push_observations(observations=observations,
                                     extra=extra)
            num_written += 1
        counter += 1

    if num_written == 0:
        msg = ('This log was too short to be written (%d observations)' % counter)
        raise Exception(msg)


@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, max_observations='>=1',
          max_time='>0')
def run_simulation_servonav(id_robot, robot, id_agent, agent,
                         max_observations, max_time,
                         id_episode, id_environment,
                   check_valid_values=True):
    ''' Runs an episode of the simulation. The agent should already been
        init()ed. '''

    keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)

    #keeper.new_episode_started(id_episode, id_environment)

    #obs_spec = robot.get_spec().get_observations()
    cmd_spec = robot.get_spec().get_commands()

    def get_observations():
        obs = robot.get_observations()

        observations = keeper.push(timestamp=obs.timestamp,
                                   observations=obs.observations,
                                   commands=obs.commands,
                                   commands_source=obs.commands_source,
                                   id_episode=id_episode,
                                   id_world=id_environment)
        episode_end = obs.episode_end
        return observations, episode_end

    counter = 0
    while True:
        observations, episode_end = get_observations()

        yield observations

        now = '%s' % counter

        if counter >= max_observations:
            logger.info('Finished at %s because %s >= %s' %
                        (now, counter, max_observations))
            break

        if observations['time_from_episode_start'] > max_time:
            logger.info('Finished at %s because of max_time: %s > %s' %
                        (now, observations['time_from_episode_start'], max_time))
            break

        if episode_end: # Fishy
            logger.info('Finished at %s because of robot driver.' % now)
            break

        agent.process_observations(observations)
        commands = agent.choose_commands() # repeated

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        robot.set_commands(commands, id_agent)

        counter += 1

