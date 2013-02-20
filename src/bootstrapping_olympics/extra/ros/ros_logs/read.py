from . import ROS2Python, logger, boot_spec_from_ros_message
from bootstrapping_olympics.logs.hints import read_hints

    

def bag_read(bag_file, topic, spec, substitute_id_episode, only_episodes=None):
    ''' 
        Reads the log in ROS bag file (only the
        topic given) and converts it to the structure we use internally.
        The ROS2Python object takes care to sanitize the logs;
        it makes lots of checks for data meant to come directly from
        real robots (so delay, wrong formats, etc. are expected. 
    '''
    from ros import rosbag  # @UnresolvedImport
    bag = rosbag.Bag(bag_file)
    
    hints = read_hints(bag_file)
    
    warned = False
    try:
        ros2python = None

        for msg in bag.read_messages(topics=[topic]):
            topic, ros_obs, t = msg  # @UnusedVariable

            if spec is None:
                spec = boot_spec_from_ros_message(ros_obs)
            if ros2python is None:
                bag_read_hints = hints.get('bag_read', {})
                if bag_read_hints:
                    logger.info('bag_read using hints: %s' % bag_read_hints)    
                ros2python = ROS2Python(spec=spec, **bag_read_hints)

            if only_episodes and not ros_obs.id_episode in only_episodes:
                continue

            if ros_obs.id_episode == 'id-episode-not-set':
                if not warned:
                    warned = True
                    # XXX
                    logger.info('Changing id_episode to %s'
                                % substitute_id_episode)
                ros_obs.id_episode = substitute_id_episode

            obs = ros2python.convert(ros_obs)
            if obs is not None:
                yield obs
    except:
        bag.close()
        raise

