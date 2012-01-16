from . import logger


# TODO: make sure that the log has extra robot_state
def create_video(data_central, id_robot, id_agent,
                 model='boot_log2movie', zoom=0, id_episode=''):
    logger.info('Creating video for robot: %r agent: %r zoom: %s' %
                (id_robot, id_agent, zoom))

    ds = data_central.get_dir_structure()

    basename = ds.get_video_basename(id_robot=id_robot,
                                     id_agent=id_agent, id_episode=id_episode)
    if zoom:
        basename += '-z%.1f' % zoom

    filename = basename + '-%s' % model
    logdir = ds.get_simulated_logs_dir()

    # TODO: check has_procgraph
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    logger.info('Writing to %r.' % filename)
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_agent=id_agent,
                id_episode=id_episode,
                zoom=zoom,
                output=filename)
    pg(model, config=config)
