from . import load_agent_state, publish_agent_output, logger
from bootstrapping_olympics import AgentInterface
from bootstrapping_olympics.utils import InAWhile, UserError
import logging
from .publish_output import publish_once as do_publish_once


def learn_log(data_central, id_agent, id_robot,
              reset=False,
              publish_interval=None,
              publish_once=False,
              interval_save=None,
              interval_print=None,
              episodes=None,
              save_state=True,
              live_plugins=[]):
    ''' If episodes is not None, it is a list of episodes id to learn. '''

    # logger.info('Learning episodes %r' % episodes)

    log_index = data_central.get_log_index()
    if not log_index.has_streams_for_robot(id_robot):
        msg = ('No log for robot %r found. I know: %s.'
               % (id_robot, ", ".join(log_index.robots2streams.keys())))
        raise Exception(msg)

    bo_config = data_central.get_bo_config()
    if not id_agent in bo_config.agents:
        msg = ('Agent %r not found in configuration. I know: %s.'
               % (id_agent, ", ".join(bo_config.agents.keys())))
        raise UserError(msg)

    live_plugins = [bo_config.live_plugins.instance(x)
                    for x in live_plugins]

    agent_logger = logging.getLogger("BO.learn:%s(%s)" % (id_agent, id_robot))
    agent_logger.setLevel(logging.DEBUG)
    AgentInterface.logger = agent_logger

    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=reset)

    db = data_central.get_agent_state_db()

    if publish_interval is not None or publish_once:
        do_publish_once(data_central, id_agent=id_agent, id_robot=id_robot,
                 phase='learn', progress='all',
                 save_pickle=False)

    if publish_once:
        logger.info('As requested, exiting after publishing information.')
        return

    streams = log_index.get_streams_for_robot(id_robot)
    # TODO: progress bar
    num_episodes_total = 0
    num_episodes_remaining = 0
    num_observations_total = 0
    num_observations_remaining = 0
    for stream in streams:
        # Check if all learned
        stream_episodes = stream.get_id_episodes()
        num_episodes_total += len(stream_episodes)
        num_observations_total += stream.get_num_observations()
        to_learn = stream_episodes.difference(state.id_episodes)
        if episodes is not None:
            to_learn = to_learn.intersection(episodes)
        if to_learn:
            num_episodes_remaining += len(to_learn)
            num_observations_remaining += stream.get_num_observations()

    template = '%20s: %7d episodes, %7d observations.'
    logger.info(template % ('total',
                            num_episodes_total,
                            num_observations_total))
    logger.info(template % ('already learned',
                            len(state.id_episodes),
                            state.num_observations))
    logger.info(template % ('remaining',
                            num_episodes_remaining,
                            num_observations_remaining))

    tracker_save = InAWhile(interval_save)

    tracker = InAWhile(interval_print)

    # Initialize plugins
    for plugin in live_plugins:
        plugin.init(dict(data_central=data_central,
                         id_agent=id_agent, id_robot=id_robot))

    for stream in streams:
        # Check if all learned
        to_learn = stream.get_id_episodes().difference(state.id_episodes)
        if episodes is not None:
            to_learn = to_learn.intersection(episodes)
        if not to_learn:
            # logger.info('Stream %s already completely learned.' % stream)
            continue

        try:
            from compmake import progress
            progress('Learning', (len(state.id_episodes),
                                  num_episodes_total))
        except ImportError:
            pass

        cur_stream_observations = 0
        for obs in stream.read(only_episodes=to_learn):
            state.num_observations += 1
            cur_stream_observations += 1

            if tracker.its_time():
                progress = 100 * (float(state.num_observations) / 
                                  num_observations_total)
                progress_log = 100 * (float(cur_stream_observations) / 
                                      stream.get_num_observations())
                msg = ('overall %.2f%% (log %3d%%) (eps: %4d/%d, obs: %4d/%d);'
                       ' %5.1f fps' % 
                       (progress, progress_log, len(state.id_episodes),
                         num_episodes_total,
                        state.num_observations, num_observations_total,
                        tracker.fps()))
                logger.info(msg)

            if save_state and tracker_save.its_time():
                logger.debug('Saving state (periodic)')
                # note: episodes not updated
                state.agent_state = agent.get_state()
                db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)

            agent.process_observations(obs)

            if publish_interval is not None:
                if 0 == state.num_observations % publish_interval:
                    publish_once(data_central, id_agent, id_robot,
                                 phase='learn', progress='%05d' % state.num_observations,
                                 save_pickle=False)
#                    
#                    publish_agent_output(state, agent, report_dir,
#                                    basename='%05d' % state.num_observations)

            # Update plugins
            for plugin in live_plugins:
                plugin.update(dict(agent=agent, robot=None, obs=obs))

        state.id_episodes.update(to_learn)

    # Saving agent state
    if save_state:
        logger.debug('Saving state (end of streams)')
        state.agent_state = agent.get_state()
        db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)

    if publish_interval is not None:
        ds = data_central.get_dir_structure()
        report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state)
        logger.info('Writing output to directory %r.' % report_dir)
        publish_agent_output(state, agent, report_dir, basename='all')

