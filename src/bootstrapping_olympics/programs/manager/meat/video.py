from . import logger


def create_video(data_central, id_robot, id_agent,
                 model='boot_log2movie', zoom=0):
    logger.info('Creating video for robot: %r agent: %r zoom: %s' %
                (id_robot, id_agent, zoom))

    ds = data_central.get_dir_structure()
    basename = ds.get_video_basename(id_robot, 'a%s' % id_agent,
                                     id_episode='all')
    if zoom:
        basename += '-z%.1f' % zoom

    filename = basename + '-%s' % model
    logdir = ds.get_simulated_logs_dir()
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    logger.info('Writing to %r.' % filename)
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_agent=id_agent,
                id_episode='',
                zoom=zoom,
                output=filename)
    pg(model, config=config)
