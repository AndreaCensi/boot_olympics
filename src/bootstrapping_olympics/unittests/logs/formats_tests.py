from ...logs import LogsFormat
from ...programs.manager.meat import simulate
from ...programs.manager.meat.data_central import DataCentral
from ...utils import assert_allclose, safe_makedirs
from ..manager_tests.test_basic_operations import create_tmp_dir
from ..tests_generation import for_all_pairs
import os


@for_all_pairs
def check_basic_operations(id_agent, agent, id_robot, robot):

    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))
        data_central = DataCentral(root)

        # Simulate two episodes
        simulate(data_central, id_agent=id_agent, id_robot=id_robot,
             max_episode_len=2,
             num_episodes=2,
             cumulative=False,
             id_episodes=None,
             stateful=False,
             interval_print=None,
             write_extra=True)

        log_index = data_central.get_log_index()
        log_index.reindex()

        stream_orig = log_index.get_streams_for(id_robot, id_agent)[0]

        for logs_format, interface in LogsFormat.formats.items():
            print('Formatting with %r' % logs_format)
            dirname = os.path.join(root, logs_format)
            safe_makedirs(dirname)
            filename = os.path.join(dirname, 'example.%s' % logs_format)
            written = []
            with interface.write_stream(filename, 'example',
                                        robot.get_spec()) as writer:
                for observations in stream_orig.read():
                    writer.push_observations(observations)
                    written.append(observations)

            count = 0
            for obs_read in interface.read_from_stream(filename, 'example'):
                original = written[count]

                assert_allclose(obs_read['timestamp'], original['timestamp'])
                assert_allclose(obs_read['observations'],
                                original['observations'])
                assert_allclose(obs_read['commands'], original['commands'])

                count += 1

        # FIXME: ROS does not allow writing extras

        # Write in every format




