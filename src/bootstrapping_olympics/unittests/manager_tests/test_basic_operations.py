from tempfile import mkdtemp
from contextlib import contextmanager
from shutil import rmtree
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from bootstrapping_olympics.programs.manager.meat import simulate
from bootstrapping_olympics.programs.manager.meat.log_learn import learn_log
from bootstrapping_olympics.utils import assert_allclose
import os
from bootstrapping_olympics.programs.manager.meat.servo import task_servo
from bootstrapping_olympics.programs.manager.meat.predict import task_predict
from bootstrapping_olympics.logs.logs_format import LogsFormat


def check_basic_operations(id_agent, id_robot):

    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))
        data_central = DataCentral(root)
        ds = data_central.get_dir_structure()
        log_index = data_central.get_log_index()

        def simulate_some(n):
            simulate(data_central, id_agent=id_agent, id_robot=id_robot,
             max_episode_len=2,
             num_episodes=n,
             cumulative=False,
             id_episodes=None,
             stateful=False,
             interval_print=None,
             write_extra=True)

        assert not log_index.has_streams_for_robot(id_robot)

        formats = LogsFormat.formats.keys()

        for logs_format in formats:
            ds.set_log_format(logs_format)
            simulate_some(2)
            simulate_some(2)


        assert not log_index.has_streams_for_robot(id_robot)
        log_index.reindex()
        assert log_index.has_streams_for_robot(id_robot)
        assert_allclose(len(log_index.get_streams_for_robot(id_robot)),
                        2 * len(formats))
        assert_allclose(len(log_index.get_streams_for(id_robot, id_agent)),
                        2 * len(formats))

        log_index.get_robot_spec(id_robot)

        assert_allclose(len(log_index.get_episodes_for_robot(id_robot)),
                        4 * len(formats))
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot, id_agent)),
                        4 * len(formats))

        learn_log(data_central, id_robot=id_robot, id_agent=id_agent)

        task_servo(data_central, id_agent, id_robot,
                   max_episode_len=1,
                   num_episodes=1,
                   displacement=1,
                   id_episodes=None,
                   cumulative=False,
                   interval_print=None,
                   num_episodes_with_robot_state=0)

        task_predict(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             interval_print=None)


def test_basic_operations_1():
    check_basic_operations(id_agent='random_agent', id_robot='random_1_4')

@contextmanager
def create_tmp_dir():
    root = mkdtemp()
    try:
        yield root
    finally:
        rmtree(root)
