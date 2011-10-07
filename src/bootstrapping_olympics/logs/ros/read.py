from . import ROS2Python

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
        ros2python = ROS2Python(spec)
        for msg in bag.read_messages(topics=[topic]):
            topic, ros_obs, t = msg #@UnusedVariable
            if ros_obs.id_episode == 'id-episode-not-set':
                if not warned:
                    warned = True
                    # XXX
                    print('Changing id_episode to %s' % substitute_id_episode)
                ros_obs.id_episode = substitute_id_episode

            obs = ros2python.convert(ros_obs)
            if obs is not None:
                yield obs
    except:
        bag.close()
        raise 
