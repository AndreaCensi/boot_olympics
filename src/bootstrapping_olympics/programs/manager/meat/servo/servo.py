from . import BookkeepingServo, run_simulation_servo
from .. import load_agent_state, logger, contract, np
from bootstrapping_olympics import LogsFormat, BootOlympicsConstants
from bootstrapping_olympics.utils import isodate_with_secs


@contract(interval_print='None|>=0')
def task_servo(data_central, id_agent, id_robot,
               max_episode_len,
               num_episodes,
               displacement,
               id_episodes=None, # if None, just use the ID given by the world
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
    robot = data_central.get_bo_config().robots.instance(id_robot)

    # TODO: check that this is a Vehicles simulation

    boot_spec = robot.get_spec()

    # Instance agent object    

    agent, _ = load_agent_state(data_central, id_agent, id_robot,
                             reset_state=False,
                             raise_if_no_state=True)

    servo_agent = agent.get_servo()

    id_agent_servo = '%s_servo' % id_agent

    ds = data_central.get_dir_structure()
    id_stream = '%s_%s_%s_servo' % (id_robot, id_agent, isodate_with_secs())
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
                                      boot_spec=boot_spec) as writer:
            counter = 0
            while bk.another_episode_todo():
                episode = robot.new_episode()

                if id_episodes is not None:
                    id_episode = id_episodes.pop(0)
                else:
                    id_episode = episode.id_episode

                save_robot_state = counter < num_episodes_with_robot_state

                servoing_episode(id_robot=id_robot, robot=robot,
                     id_servo_agent=id_agent_servo, servo_agent=servo_agent,
                     writer=writer, id_episode=id_episode,
                     displacement=displacement,
                     max_episode_len=max_episode_len,
                     save_robot_state=save_robot_state,
                     max_tries=10000)

                bk.episode_done()
                counter += 1


def servoing_episode(id_robot, robot,
                     id_servo_agent, servo_agent,
                     writer, id_episode, displacement,
                     max_episode_len, save_robot_state,
                     max_tries=10000):
    '''
    
        :arg:displacement: Time in seconds to displace the robot.
    '''
    from geometry import SE3

    def mean_observations(n=10):
        obss = []
        for _ in range(n): # XXX: fixed threshold
            obss.append(robot.get_observations().observations)
        return np.mean(obss, axis=0)

    def robot_pose():
        return robot.get_observations().robot_pose

    def timestamp():
        return robot.get_observations().timestamp

    def episode_ended():
        return robot.get_observations().episode_end

    def simulate_hold(cmd0, displacement):
        t0 = timestamp()
        nsteps = 0
        while timestamp() < t0 + displacement:
            nsteps += 1
            source = BootOlympicsConstants.CMD_SOURCE_SERVO_DISPLACEMENT
            robot.set_commands(cmd0, source)
            if episode_ended():
                logger.debug('Collision after %d steps' % ntries)
                return False

        logger.debug('%d steps of simulation to displace by %s' %
                    (nsteps, displacement))
        return True

    for ntries in xrange(max_tries):
        # iterate until we can do this correctly
        episode = robot.new_episode()
        obs0 = mean_observations()
        cmd0 = robot.get_spec().get_commands().get_random_value()
        pose0 = robot_pose()
        ok = simulate_hold(cmd0, displacement)
        if ok:
            pose1 = robot_pose()
            logger.info('Displacement after %s tries.' % ntries)
            break
    else:
        msg = 'Could not do the displacement (%d tries).' % max_tries
        raise Exception(msg)

    servo_agent.set_goal_observations(obs0)

    for robot_observations, boot_observations in \
        run_simulation_servo(id_robot, robot,
                           id_servo_agent, servo_agent,
                           100000, max_episode_len,
                           id_episode=id_episode,
                           id_environment=episode.id_environment):

        def pose_to_yaml(x):
            ''' Converts to yaml, or sets None. '''
            if x is None:
                return None
            else:
                return SE3.to_yaml(x)

        current_pose = robot_observations.robot_pose

        extra = {}

        extra['servoing_base'] = dict(goal=obs0.tolist(),
                                current=boot_observations['observations'].tolist())

        extra['servoing_poses'] = dict(goal=pose_to_yaml(pose0),
                                       current=pose_to_yaml(current_pose))

        # TODO: make it not overlapping
        extra['servoing'] = dict(obs0=obs0.tolist(),
                                pose0=pose_to_yaml(pose0),
                                poseK=pose_to_yaml(current_pose),
                                obsK=boot_observations['observations'].tolist(),
                                displacement=displacement,
                                cmd0=cmd0.tolist(),
                                pose1=pose_to_yaml(pose1))

        if save_robot_state:
            extra['robot_state'] = robot.get_state()

        writer.push_observations(observations=boot_observations,
                                 extra=extra)
#    
#    # TODO: add intrinsic notion here
#    @contract(returns='SE2')
#    def random_displacement():
#        # ok for rf, cam
#        # max_angle = np.deg2rad(15)
#        #max_t = 0.3
#        # ok for rf, cam
##        max_angle = np.deg2rad(35)
##        max_t = 0.1
#        # 2,30 rf
#        max_t = 0.2
#        max_angle = np.deg2rad(35)
#        # t = max_t * np.random.uniform(-max_t, +max_t, 2)
#        # theta = np.random.uniform(-max_angle, +max_angle)
#        phi = np.random.rand() * 2 * np.pi
#        x = np.cos(phi) * max_t
#        y = np.sin(phi) * max_t
#        t = np.array([x, y])
#        theta = np.sign(np.random.randn()) * max_angle
#        
#        return SE2_from_translation_angle(t, theta)
#         
#                # OK this only works with Vehicles
#                vehicle = robot.vehicle
#                obss = []
#                # average observations
#                for _ in range(10): # XXX: fixed threshold
#                    obss.append(robot.get_observations().observations)
#                obs0 = np.mean(obss, axis=0)
#                pose0 = vehicle.get_pose()
#                displ = SE3_from_SE2(random_displacement())
#                pose1 = SE3.multiply(pose0, displ)
#                vehicle.set_pose(pose1)
#                servo_agent.set_goal_observations(obs0)
#                     
#                for observations in run_simulation_servo(id_robot, robot,
#                                       id_agent_servo, servo_agent,
#                                       100000, max_episode_len,
#                                       id_episode=id_episode,
#                                       id_environment=episode.id_environment):            
#                    bk.observations(observations)
#                    
#                    servoing = dict(obs0=obs0.tolist(),
#                                    pose0=to_yaml('SE3', pose0),
#                                    displ=to_yaml('SE3', displ),
#                                    pose1=to_yaml('SE3', pose1))
#                    extra = dict(servoing=servoing)
#                    if counter < num_episodes_with_robot_state:
#                        extra['robot_state'] = robot.get_state()
#
#                    writer.push_observations(observations=observations,
#                                             extra=extra)




