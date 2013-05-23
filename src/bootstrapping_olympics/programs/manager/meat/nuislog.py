from bootstrapping_olympics import LogsFormat, logger
from bootstrapping_olympics.library.robots import EquivRobot
from bootstrapping_olympics.utils import UserError
from contracts import describe_type
import warnings

__all__ = ['task_predict', 'predict_report']

def add_dummy_robots(data_central):
    log_index = data_central.get_log_index()
    config = data_central.get_bo_config()
    
    for id_robot in log_index.list_robots():
        boot_spec = log_index.get_robot_spec(id_robot)
        boot_spec_yaml = boot_spec.to_yaml()
        conf = {'id': id_robot, 'desc': '',
                'code': ['bootstrapping_olympics.library.robots.dummy_robot_from_spec',
                         {'boot_spec_yaml': boot_spec_yaml}]}
        warnings.warn('check its not there yet?') 
        config.robots[id_robot] = conf     
        

def task_nuislog(data_central, id_equiv, id_robot, with_extra=False):
    log_index = data_central.get_log_index()

    add_dummy_robots(data_central)
    
    # First, look for all episodes of id_robot
    streams = log_index.get_streams_for_robot(id_robot)
    logger.info('Found %d streams for robot %r' % (len(streams), id_robot))
    if len(streams) == 0:
        msg = 'No existing logs for %r' % id_robot
        raise UserError(msg)
    
    # Instance the equiv robot
    config = data_central.get_bo_config()
    equiv = config.robots.instance(id_equiv)
    
    if not isinstance(equiv, EquivRobot):
        msg = ('I expect the robot %r to be an EquivRobot robot '
               'but found %s.' % (id_equiv, describe_type(equiv))) 
        raise UserError(msg)
    
    assert isinstance(equiv, EquivRobot)

    if equiv.inner_robot_name != id_robot:
        msg = ('The equiv robot inner robot is %r, but you requested %r'
               % (equiv.inner_robot_name, id_robot))
        raise UserError(msg) 
    
    # FIXME: this is going to work only for 1 nuisance    
    for stream in streams:
        logger.info('Converting %r' % stream)
        nuislog_stream(data_central, stream, id_equiv,
                       equiv, with_extra=with_extra)
        

def nuislog_stream(data_central, stream, id_equiv, equiv, with_extra):
    ds = data_central.get_dir_structure()
    id_agent = list(stream.get_id_agents())[0]
    id_stream, filename = ds.get_simlog_filename_stream(id_robot=id_equiv,
                                                        id_agent=id_agent)
    logs_format = LogsFormat.get_reader_for(filename)

    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=equiv.get_spec()) as writer:
        for obs1 in stream.read(read_extra=with_extra):
            obs2 = equiv.convert_observations_array(obs1)
            obs2['id_robot'] = id_equiv
            extra = obs1['extra'].item()
            if not with_extra:
                extra = {}
            writer.push_observations(observations=obs2, extra=extra)


def nuislog_episodes(data_central, id_robot_original, id_episodes,
                     id_equiv, obs_nuisances, cmd_nuisances, with_extras=True):
    
    equiv = EquivRobot(robot=id_robot_original,
                       obs_nuisance=list(obs_nuisances),
                       cmd_nuisance=list(cmd_nuisances))
    
    log_index = data_central.get_log_index()
    ds = data_central.get_dir_structure()
    id_stream, filename = ds.get_simlog_filename_stream(id_robot=id_equiv,
                                                        id_agent='derived')
    logs_format = LogsFormat.get_reader_for(filename)
    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=equiv.get_spec()) as writer:
        for id_episode in id_episodes:
            for obs1 in log_index.read_robot_episode(id_robot_original,
                                                     id_episode, read_extra=with_extras):
                obs2 = equiv.convert_observations_array(obs1)
                obs2['id_robot'] = id_equiv
                extra = obs1['extra'].item()
                writer.push_observations(observations=obs2, extra=extra)
                
