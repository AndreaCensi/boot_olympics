from . import logger


# TODO: make sure that the log has extra robot_state
def create_video(data_central, id_robot, id_agent,
                 model='boot_log2movie',
                 model_params={}, #{'plotter.zoom': 2},
                 suffix='',
                 id_episode=''):

    logger.info('Creating video for robot: %r agent: %r episode %r' %
                (id_robot, id_agent, id_episode))

    ds = data_central.get_dir_structure()

    basename = ds.get_video_basename(id_robot=id_robot,
                                     id_agent=id_agent,
                                     id_episode=id_episode)
    if suffix:
        basename += '-%s' % suffix

    logdir = ds.get_simulated_logs_dir()

    # TODO: check has_procgraph
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    logger.info('Writing to %r.' % basename)
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_agent=id_agent,
                id_episode=id_episode,
                output=basename,
                **model_params)
    pg(model, config=config, stats=False)

    ds.file_is_done(basename, desc="")

