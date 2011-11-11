from . import logger, np
from geometry import SE2, SE2_from_SE3, translation_from_SE2, angle_from_SE2
from reprep import Report # TODO: be safe
import geometry
import os
from bootstrapping_olympics.utils.pylab_axis import x_axis_balanced


def servo_stats_summaries(data_central, id_agent, id_robot):
    log_index = data_central.get_log_index()
    episodes = log_index.get_episodes_for_robot(id_robot, id_agent + '_servo')
    if not episodes:
        raise Exception('No episodes found for %s - %s' % (id_robot, id_agent))
    summaries = []
    for id_episode in episodes:
        logger.info('Reading %s...' % id_episode)
        
        errors = []
        timestamps = []
        poses = []
        for observations in log_index.read_robot_episode(id_robot, id_episode,
                                                         read_extra=True):
            servoing = observations['extra'].item()['servoing']
            
            obs0 = np.array(servoing['obs0'])
            pose0 = SE2_from_SE3(geometry.yaml.from_yaml(servoing['pose0']))
            pose1 = SE2_from_SE3(geometry.yaml.from_yaml(servoing['pose1']))
            poseK = SE2_from_SE3(geometry.yaml.from_yaml(servoing['poseK']))
            
            pose1 = SE2.multiply(SE2.inverse(pose0), pose1)
            poseK = SE2.multiply(SE2.inverse(pose0), poseK)
            poses.append(poseK)
            #obs1 = np.array(servoing['obs1'])
            obsK = np.array(servoing['obsK'])
            
            err_L2 = np.linalg.norm(obs0 - obsK)
            
            errors.append(err_L2)
            timestamps.append(observations['timestamp'])
#            last['time_from_episode_start'] = observations['time_from_episode_start']
            
        summary = {}
        summary['pose0'] = pose0
        summary['pose1'] = pose1
        summary['poses'] = poses
        summary['errors'] = errors
        summary['timestamps'] = timestamps
        summary['initial_translation'] = translation_from_SE2(pose1)
        summary['initial_distance'] = np.linalg.norm(translation_from_SE2(pose1))
        summary['initial_rotation'] = angle_from_SE2(pose1)
        
        summaries.append(summary)
        
    return summaries
    
def servo_stats_report(data_central, id_agent, id_robot, summaries):
    if not summaries:
        raise Exception('Empty summaries')
        
    def extract(key):
        return np.array([s[key] for s in summaries])
    
    initial_distance = extract('initial_distance')
    initial_rotation = extract('initial_rotation')
        
    basename = 'servo_analysis-%s-%s' % (id_agent, id_robot)
    r = Report(basename)
    
    f = r.figure(cols=1)
    with f.data_pylab('errors') as pylab:
        for summary in summaries:
            errors = summary['errors']
            pylab.plot(errors)

    with f.data_pylab('initial_rotation') as pylab:
        pylab.hist(np.rad2deg(initial_rotation))
        x_axis_balanced(pylab)
        
    with f.data_pylab('initial_distance') as pylab:
        pylab.hist(initial_distance)
     
    
    ds = data_central.get_dir_structure()
    report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state='servo_stats',
                                       phase='servo_stats')
    filename = os.path.join(report_dir, 'servo_stats_report.html')
    logger.info('Writing output to %r.' % filename)
    r.to_html(filename, resources_dir=os.path.join(report_dir, 'images'))
        
        
