from tempfile import mkdtemp
from contextlib import contextmanager
from shutil import rmtree
from bootstrapping_olympics.programs.manager.meat.data_central import DataCentral
from geometry.basic_utils import assert_allclose
import os
from bootstrapping_olympics.programs.manager.command_line.main import boot_olympics_manager
from bootstrapping_olympics.unittests.tests_generation import for_all_pairs

@for_all_pairs
def check_basic_operations_cmdline(id_agent, agent, id_robot, robot):
    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config')) # XXX make it automatic
        data_central = DataCentral(root)
        log_index = data_central.get_log_index()
        
        def execute_command(*args):
            arguments = ['-d', root] + list(args)
            boot_olympics_manager(arguments)
        
        
            
        
        assert not log_index.has_streams_for_robot(id_robot)
                  
        execute_command('--logformat', 'h5',
                        'simulate', '-a', id_agent, '-r', id_robot,
                         '--num_episodes', '2', '--episode_len', '2')
        execute_command('--logformat', 'h5',
                        'simulate', '-a', id_agent, '-r', id_robot,
                         '--num_episodes', '2', '--episode_len', '2')
        
        execute_command('--logformat', 'bag',
                        'simulate', '-a', id_agent, '-r', id_robot,
                         '--num_episodes', '2', '--episode_len', '2')
        execute_command('--logformat', 'bag',
                        'simulate', '-a', id_agent, '-r', id_robot,
                         '--num_episodes', '2', '--episode_len', '2')
        
        assert not log_index.has_streams_for_robot(id_robot)
        log_index.reindex()
        assert log_index.has_streams_for_robot(id_robot)
        assert_allclose(len(log_index.get_streams_for_robot(id_robot)), 4) 
        assert_allclose(len(log_index.get_streams_for(id_robot, id_agent)), 4)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot)), 8)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot, id_agent)), 8)
        

        execute_command('learn-log', '-a', id_agent, '-r', id_robot)
        execute_command('servo', '-a', id_agent, '-r', id_robot,
                        '--num_episodes', '1',
                        '--max_episode_len', '1')
        
        execute_command('predict', '-a', id_agent, '-r', id_robot)

        execute_command('list-logs')
        execute_command('list-logs', '-e')
        execute_command('list-logs', '-l')
        execute_command('list-logs', '-s')
        
        execute_command('list-logs', '-R')

        # TODO: publish
        #execute_command('batch') # TODO: test batch
        

        
        execute_command('list-agents')
        execute_command('list-agents', '-v')
        execute_command('list-robots')
        execute_command('list-robots', '-v')
        execute_command('list-states')
        execute_command('list-states', '-v')
        
@contextmanager
def create_tmp_dir():
    root = mkdtemp()
    try: 
        yield root
    finally:
        rmtree(root) 
