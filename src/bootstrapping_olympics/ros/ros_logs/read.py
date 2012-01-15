from . import boot_spec_from_ros_message, logger, np
from ... import ObsKeeper, BootInvalidValue


def bag_read(bag_file, topic, spec, substitute_id_episode):
    ''' 
        Reads the log in ROS bag file (only the
        topic given) and converts it to the structure we use internally.
        The ROS2Python object takes care to sanitize the logs;
        it makes lots of checks for data meant to come directly from
        real robots (so delay, wrong formats, etc. are expected. 
    '''
    from ros import rosbag #@UnresolvedImport
    bag = rosbag.Bag(bag_file)
    warned = False
    try:
        ros2python = None

        for msg in bag.read_messages(topics=[topic]):
            topic, ros_obs, t = msg #@UnusedVariable

            if spec is None: spec = boot_spec_from_ros_message(ros_obs)
            if ros2python is None: ros2python = ROS2Python(spec=spec)


            if ros_obs.id_episode == 'id-episode-not-set':
                if not warned:
                    warned = True
                    # XXX
                    logger.info('Changing id_episode to %s' % substitute_id_episode)
                ros_obs.id_episode = substitute_id_episode

            obs = ros2python.convert(ros_obs)
            if obs is not None:
                yield obs
    except:
        bag.close()
        raise


class ROS2Python():
    ''' 
        This class takes converts a BootstrappingObservations ROS structure
        into an Observation type object. It also makes some consistency
        checks, marks different episodes, discards repeated observations, etc. 
        The idea is that the data that comes out of this will not need any 
        other check by the agents. 
    '''

    def __init__(self, spec, max_dt=1):
        self.last = None
        self.spec = spec
        self.max_dt = max_dt

        self.keeper = None

    def convert(self, ros_obs, filter_doubles=True):
        ''' Returns None if the same message is repeated. '''

        if self.keeper is None:
            self.keeper = ObsKeeper(self.spec, ros_obs.id_robot)

        if filter_doubles:
            if self.last is not None and self.last['counter'] == ros_obs.counter:
                return None

        current_data_description = ('[%s#%s at %s]' %
                    (ros_obs.id_episode, ros_obs.counter, ros_obs.timestamp))

        # FIXME: this assumes we use floats 
        sensel_values_reshaped = np.array(ros_obs.sensel_values, dtype='float32')
        sensel_values_reshaped = sensel_values_reshaped.reshape(ros_obs.sensel_shape)
        commands = np.array(ros_obs.commands)
        try:
            self.spec.get_commands().check_valid_value(commands)
            self.spec.get_observations().check_valid_value(sensel_values_reshaped)
        except BootInvalidValue as e:
            logger.error('%s: Skipping invalid data.' % current_data_description)
            logger.error(e)
            return None

#        obs = Observations()
#        obs.time = ros_obs.timestamp
#        obs.sensel_values = sensel_values_reshaped
#        obs.commands = np.array(ros_obs.commands, dtype='float32')
#        obs.commands_source = ros_obs.commands_source
#        obs.counter = ros_obs.counter
#        obs.id_episode = ros_obs.id_episode
#        # obs.dt is useless

        # FIXME: we have to do this again :-/
        # FIXME: at least, filter the doubles and max_dt

#        if self.last_ros_obs is not None:
#            obs.dt = obs.time - self.last.time
#           
#            obs.episode_changed = obs.id_episode != self.last.id_episode
#            if not obs.episode_changed:
#                #assert obs.dt >= 0
#                if obs.dt < 0:
#                    logger.error('At %s' % current_data_description)
#                    logger.error('Out of order? previous time: %s '
#                                 'current: %s dt: %s' % 
#                                 (self.last.time, obs.time, obs.dt))
#                    return None
#                if filter_doubles:
#                    # assert obs.dt > 0
#                    if obs.dt <= 0:
#                        logger.error('At %s' % current_data_description)
#                        logger.error('Strange, should have caught before.')
#                        return None    
#                
#                if obs.dt > self.max_dt:
#                    logger.info('Skipping %s due to strange dt %s .' % 
#                                (current_data_description, obs.dt))
#            else:
#                obs.dt = 0.01 # episode changed
#        else:
#            obs.dt = 0.01
#            obs.episode_changed = True

        self.last = self.keeper.push(
                timestamp=ros_obs.timestamp,
                observations=sensel_values_reshaped,
                commands=commands,
                commands_source=ros_obs.commands_source,
                id_episode=ros_obs.id_episode,
                id_world=ros_obs.id_environment)

        self.last_ros_obs = ros_obs

        return self.last
