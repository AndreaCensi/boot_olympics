from . import default_explorer
from .utils import create_tmp_dir
from boot_manager import DataCentral
from boot_manager.logs.logs_format import LogsFormat
from boot_manager.meat.load_agent_state_imp import load_agent_state
from boot_manager.programs.manager.command_line.main import manager_main
from bootstrapping_olympics import (ExploringAgent, LearningAgent, 
    PredictingAgent, ServoingAgent, UnsupportedSpec, logger)
from bootstrapping_olympics.unittests import for_all_pairs
from bootstrapping_olympics.utils import assert_allclose
from comptests import PartiallySkipped, Skipped
from contracts import describe_type
import os


# TODO: check that the robot generates different episodes strings
@for_all_pairs
def check_cmdline(id_agent, agent, id_robot, robot):  # @UnusedVariable
    if not isinstance(agent, ExploringAgent):
        id_explorer = default_explorer
        print('%s not exploring agent: %s' % (id_agent, describe_type(agent)))
        if not isinstance(agent, LearningAgent):
            msg = 'Agent is neither Learning or Exploring (%s): %s' % (id_agent,describe_type(agent))
            raise Exception(msg)
    else:
        id_explorer = id_agent
    
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return Skipped('UnsupportedSpec')
    
    with create_tmp_dir() as root:
        os.mkdir(os.path.join(root, 'config'))  # XXX make it automatic
        data_central = DataCentral(root)
        log_index = data_central.get_log_index()

        def execute_command(*args):
            arguments = ['-d', root, '--contracts'] + list(args)
            res = manager_main(args=arguments, sys_exit=False)
            print(res)
            if res:
                msg = 'manager failed\n args %s\n res: %s' % (arguments, res)
                raise ValueError(msg)

        assert not log_index.has_streams_for_robot(id_robot)
        formats = LogsFormat.formats.keys()

        for logs_format in formats:
            execute_command('--logformat', logs_format,
                            'simulate', '-a', id_explorer, '-r', id_robot,
                             '--num_episodes', '2', '--episode_len', '2')
            execute_command('--logformat', logs_format,
                            'simulate', '-a', id_explorer, '-r', id_robot,
                             '--num_episodes', '2', '--episode_len', '2')

        assert not log_index.has_streams_for_robot(id_robot)
        log_index.reindex()
        assert log_index.has_streams_for_robot(id_robot)
        n = len(formats)
        assert_allclose(len(log_index.get_streams_for_robot(id_robot)), 2 * n)
        assert_allclose(len(log_index.get_streams_for(id_robot,
                                                      id_explorer)), 2 * n)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot)), 4 * n)
        assert_allclose(len(log_index.get_episodes_for_robot(id_robot,
                                                             id_explorer)), 4 * n)
        
        
        
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
        
        if not isinstance(agent, LearningAgent):
            if not isinstance(agent, ExploringAgent):
                msg = 'Agent is neither Learning or Exploring (%s)' % id_agent
                raise Exception(msg)
            return PartiallySkipped(['learn','servo','predict'])

        else: 
            execute_command('learn-log', '-a', id_agent, '-r', id_robot)
    
    
            agent2, _ =  load_agent_state(data_central, id_agent, id_robot,
                     reset_state=False,
                     raise_if_no_state=True)
            
            skipped = []
    
            if isinstance(agent2, ServoingAgent):
                try: 
                    agent2.get_servo()
                except NotImplementedError:
                    skipped.append('servo')
                    pass  
                else:  
                    execute_command('servo', '-a', id_agent, '-r', id_robot,
                            '--num_episodes', '1',
                            '--max_episode_len', '1')
            else:
                skipped.append('servo')
                
                
            if True:
                logger.error('-'*50 + '\n task_predict disabled \n') #XXX: FIXME
               
            else:
                if isinstance(agent2, PredictingAgent):
                    try: 
                        agent2.get_predictor()
                    except NotImplementedError:
                        skipped.append('predict')
                        pass  
                    else:
                        execute_command('predict', '-a', id_agent, '-r', id_robot)
                        
                else:
                    skipped.append('predict')
        
    
        if skipped:
            return PartiallySkipped(skipped)
        
