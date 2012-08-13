from . import logger
from bootstrapping_olympics.logs import LogsFormat
from bootstrapping_olympics.utils import UserError
from contracts import describe_type
from bootstrapping_olympics.library.robots.equiv_robot import EquivRobot

__all__ = ['task_predict', 'predict_report']



def task_nuislog(data_central, id_equiv, id_robot):

    # First, look for all episodes of id_robot
    log_index = data_central.get_log_index()
    streams = log_index.get_streams_for_robot(id_robot)
    logger.info('Found %d streams for robot %r' % (len(streams), id_robot))
    if len(streams) == 0:
        msg = 'No existing logs for %r' % id_robot
        raise UserError(msg)
    
    config = data_central.get_bo_config()
    equiv = config.robots.instance(id_equiv)
    
    if not isinstance(equiv, EquivRobot):
        msg = ('I expect the robot %r to be a virtual robot '
               'but found %s.' % (id_equiv, describe_type(equiv))) 
        raise UserError(msg)
    
    assert isinstance(equiv, EquivRobot)

    
    if equiv.inner_robot_name != id_robot:
        msg = ('The equiv robot inner robot is %r, but you requested %r'
               % (equiv.inner_robot_name, id_robot))
        raise UserError(msg) 
    
    # FIXME: this is going to work only for 1 nuisance
    
    for stream in streams:
        nuislog_stream(data_central, stream, id_equiv, equiv, with_extra=True)
        

def nuislog_stream(data_central, stream, id_equiv, equiv, with_extra):
    logger.info('Converting %r' % stream)
    ds = data_central.get_dir_structure()
    id_agent = list(stream.get_id_agents())[0]
    id_stream, filename = ds.get_simlog_filename_stream(id_robot=id_equiv,
                                             id_agent=id_agent)
    logs_format = LogsFormat.get_reader_for(filename)

    with logs_format.write_stream(filename=filename, id_stream=id_stream,
                                  boot_spec=stream.get_spec()) as writer:
        for obs1 in stream.read(read_extra=with_extra):
            obs2 = equiv.convert_observations_array(obs1)
            obs2['id_robot'] = id_equiv
            extra = obs1['extra'].item()
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
                
            


   
    
    
