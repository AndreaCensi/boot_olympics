import os

from bootstrapping_olympics import LogsFormat, logger, UnsupportedSpec
from bootstrapping_olympics.programs.manager import DataCentral, simulate
from bootstrapping_olympics.utils import assert_allclose, safe_makedirs

from ..manager_tests import create_tmp_dir
from ..tests_generation import for_all_pairs
from bootstrapping_olympics.interfaces.agent import ExploringAgent


@for_all_pairs
def check_logs_formats(id_agent, agent, id_robot, robot):  # @UnusedVariable
    if not isinstance(agent, ExploringAgent):
        print('skipping because agent is not active')
        return dict(result='skip')

    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))
        data_central = DataCentral(root)

        # Simulate two episodes
        # NO! there is a bug in bag reading; the messages are read
        # in timestamp order; and for now different episodes can
        # have overlapping timestamps
        try:
            simulate(data_central, id_agent=id_agent, id_robot=id_robot,
                 max_episode_len=2,
                 num_episodes=1,  # changed from 2 (see above)
                 cumulative=False,
                 id_episodes=None,
                 stateful=False,
                 interval_print=None,
                 write_extra=True)
        except UnsupportedSpec:
            return

        log_index = data_central.get_log_index()
        log_index.reindex()

        streams = log_index.get_streams_for(id_robot, id_agent)
        if len(streams) != 1:
            msg = 'Expected to find 1 stream, not %d' % len(streams)
            raise Exception(msg)

        stream_orig = streams[0]

        for logs_format, interface in LogsFormat.formats.items():
            try:
                dirname = os.path.join(root, logs_format)
                safe_makedirs(dirname)
                filename = os.path.join(dirname, 'example.%s' % logs_format)
                written = []
                id_stream = 'example'
                with interface.write_stream(filename, id_stream,
                                            robot.get_spec()) as writer:
                    for observations in stream_orig.read():
                        logger.info('Writing %s:%s (%s)' % 
                              (observations['id_episode'],
                               observations['counter'],
                               observations['timestamp']))
                        writer.push_observations(observations)
                        written.append(observations)

                count = 0
                for obs_read in interface.read_from_stream(filename,
                                                           id_stream):
                    logger.info('Reading %s:%s (%s)' % 
                          (obs_read['id_episode'],
                           obs_read['counter'],
                           obs_read['timestamp']))

                    original = written[count]

                    try:
                        if obs_read['counter'] != original['counter']:
                            msg = ('Not even the counter is the same!'
                                   ' %s vs %s' % 
                                   (obs_read['counter'], original['counter']))
                            raise Exception(msg)

                        assert_allclose(obs_read['timestamp'],
                                        original['timestamp'])
                        assert_allclose(obs_read['observations'],
                                        original['observations'])
                        assert_allclose(obs_read['commands'],
                                        original['commands'])
                    except:
                        logger.error('Error at count = %d' % count)
                        logger.error('  original: %s' % original)
                        logger.error('  obs_read: %s' % obs_read)
                        raise
                    count += 1

                if count != len(written):
                    msg = ('I wrote %d entries, but obtained %d.' % 
                           (len(written), count))
                    raise Exception(msg)
            except:
                logger.error('Could not pass tests for format %r.'
                             % logs_format)
                raise
        # FIXME: ROS does not allow writing extras

        # Write in every format




