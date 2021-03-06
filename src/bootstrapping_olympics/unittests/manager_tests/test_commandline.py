from .utils import create_tmp_dir
from bootstrapping_olympics import LogsFormat, UnsupportedSpec
from bootstrapping_olympics.programs.manager import (boot_olympics_manager,
    DataCentral)
from bootstrapping_olympics.unittests import for_all_pairs
from bootstrapping_olympics.utils import assert_allclose
import os

# TODO: check that the robot generates different episodes strings


@for_all_pairs
def check_cmdline(id_agent, agent, id_robot, robot):  # @UnusedVariable
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return
    
    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))  # XXX make it automatic
        data_central = DataCentral(root)
        log_index = data_central.get_log_index()

        def execute_command(*args):
            arguments = ['-d', root, '--contracts'] + list(args)
            boot_olympics_manager(arguments)

        assert not log_index.has_streams_for_robot(id_robot)
        formats = LogsFormat.formats.keys()

        for logs_format in formats:
            execute_command('--logformat', logs_format,
                            'simulate', '-a', id_agent, '-r', id_robot,
                             '--num_episodes', '2', '--episode_len', '2')
            execute_command('--logformat', logs_format,
                            'simulate', '-a', id_agent, '-r', id_robot,
                             '--num_episodes', '2', '--episode_len', '2')

        assert not log_index.has_streams_for_robot(id_robot)
        log_index.reindex()
        assert log_index.has_streams_for_robot(id_robot)
        n = len(formats)
        assert_allclose(len(log_index.get_streams_for_robot(id_robot)), 2 * n)
        assert_allclose(len(log_index.get_streams_for(id_robot,
                                                      id_agent)), 2 * n)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot)), 4 * n)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot,
                                                             id_agent)), 4 * n)

        execute_command('learn-log', '-a', id_agent, '-r', id_robot)

        
        try:
            agent.get_servo()
        except NotImplementedError:
            pass
        else:         
            execute_command('servo', '-a', id_agent, '-r', id_robot,
                                '--num_episodes', '1',
                                '--max_episode_len', '1')

        try:
            agent.get_predictor()
        except NotImplementedError:
            pass
        else:         
            execute_command('predict', '-a', id_agent, '-r', id_robot)
    
        execute_command('list-logs')
        execute_command('list-logs', '-e')
        execute_command('list-logs', '-l')
        execute_command('list-logs', '-s')

        execute_command('list-logs', '-R')

        # TODO: publish
        # execute_command('batch') # TODO: test batch 

        execute_command('list-agents')
        execute_command('list-agents', '-v')
        execute_command('list-robots')
        execute_command('list-robots', '-v')
        execute_command('list-states')
        execute_command('list-states', '-v')
