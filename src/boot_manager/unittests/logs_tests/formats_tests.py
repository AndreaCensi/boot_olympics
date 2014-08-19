from ..manager_tests import create_tmp_dir
from bootstrapping_olympics import (ExploringAgent, UnsupportedSpec, 
    logger)
from bootstrapping_olympics.utils import assert_allclose, safe_makedirs
from bootstrapping_olympics.utils.dates import unique_timestamp_string
from comptests.results import Skipped
import os
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics.unittests.tests_generation import for_all_agent_robot
from boot_manager.logs.data_central import DataCentral
from boot_manager.meat.simulate_imp import simulate_agent_robot
from boot_manager.logs.logs_format import LogsFormat




@for_all_agent_robot
def check_logs_formats(id_agent, agent, id_robot, robot):  # @UnusedVariable
    if not isinstance(agent, ExploringAgent):
        return Skipped('agent not ExploringAgent')

    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))
        data_central = DataCentral(root)

        # Simulate two episodes
        # NO! there is a bug in bag reading; the messages are read
        # in timestamp order; and for now different episodes can
        # have overlapping timestamps
        try:
            num_episodes=1 # changed from 2 (see above)
            id_episodes = [(unique_timestamp_string() +'-%s' % i) 
                           for i in range(num_episodes)]

            simulate_agent_robot(data_central, id_agent=id_agent, id_robot=id_robot,
                 max_episode_len=2,
                 cumulative=False,
                 id_episodes=id_episodes,
                 stateful=False,
                 write_extra=True)
        except UnsupportedSpec:
            return Skipped('UnsupportedSpec')


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
                                            robot.get_spec(), id_robot=id_robot,
                                            id_agent=id_agent) as writer:
                    writer.reset()
                    for x in stream_orig.read():
                        check_timed_named(x)
                        t, (s, v) = x
                        writer.put((t, (s,v)))

                        written.append((t, (s,v)))

                count = 0
                for x in interface.read_from_stream(filename, id_stream):
                    check_timed_named(x)
                    t, (s, v) = x
                    original = written[count]

                    try:
                        assert x[0] == original[0]
                        assert x[1][0] == original[1][0]
                        if isinstance(x[1][1], np.ndarray):
                            assert_allclose(x[1][1],original[1][1])
                        else:
                            assert x[1][1] == original[1][1]
                    except:
                        logger.error('Error at count = %d' % count)
                        logger.error('  original: %s' % str(original))
                        logger.error('  obs_read: %s' % str(x))
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

import numpy as np


