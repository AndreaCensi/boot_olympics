from . import logger, np


def servo_stats_summaries(data_central, id_agent, id_robot, id_episodes=None):
    from geometry import (SE2, SE2_from_SE3, translation_from_SE2, angle_from_SE2, SE3)
    log_index = data_central.get_log_index()

    if id_episodes is None:
        id_episodes = log_index.get_episodes_for_robot(id_robot,
                                                    id_agent + '_servo')
        if not id_episodes:
            msg = 'No episodes found for %s - %s' % (id_robot, id_agent)
            raise Exception(msg)
    summaries = []
    for id_episode in id_episodes:
        logger.debug('Reading %s...' % id_episode)

        errors = []
        timestamps = []
        poses = []
        dist_xy = []
        dist_th = []
        for observations in \
            log_index.read_robot_episode(id_robot, id_episode,
                                         read_extra=True):
            extra = observations['extra'].item()

            servoing = extra.get('servoing', None)
            if servoing is None:
                logger.error('Warning, no "servoing" in episode %r.' %
                              id_episode)
                break

            obs0 = np.array(servoing['obs0'])
            pose0 = SE2_from_SE3(SE3.from_yaml(servoing['pose0']))
            pose1 = SE2_from_SE3(SE3.from_yaml(servoing['pose1']))
            poseK = SE2_from_SE3(SE3.from_yaml(servoing['poseK']))

            pose1 = SE2.multiply(SE2.inverse(pose0), pose1)
            poseK = SE2.multiply(SE2.inverse(pose0), poseK)
            poses.append(poseK)

            dist_xy.append(np.linalg.norm(translation_from_SE2(poseK)))
            dist_th.append(np.abs(angle_from_SE2(poseK)))


            #obs1 = np.array(servoing['obs1'])
            obsK = np.array(servoing['obsK'])


            err_L2 = np.linalg.norm(obs0 - obsK)

            errors.append(err_L2)
            timestamps.append(observations['timestamp'])
# last['time_from_episode_start'] = observations['time_from_episode_start']

        initial_distance = np.linalg.norm(translation_from_SE2(pose1))

        summary = {}
        summary['pose0'] = pose0
        summary['pose1'] = pose1
        summary['poses'] = poses
        summary['errors'] = errors
        summary['timestamps'] = timestamps
        summary['initial_translation'] = translation_from_SE2(pose1)
        summary['initial_distance'] = initial_distance
        summary['initial_rotation'] = angle_from_SE2(pose1)
        summary['dist_xy'] = dist_xy
        summary['dist_th'] = dist_th

        summaries.append(summary)

    return summaries
