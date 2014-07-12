from .utils import create_tmp_dir
from bootstrapping_olympics import LogsFormat, UnsupportedSpec, logger
from bootstrapping_olympics.interfaces.agent import (ExploringAgent, 
    PredictingAgent, ServoingAgent)
from bootstrapping_olympics.programs.manager.meat import (
    DataCentral, learn_log, simulate, task_predict, task_servo)
from bootstrapping_olympics.unittests import for_all_pairs
from contracts.utils import describe_type
from comptests import PartiallySkipped, Skipped
import os




@for_all_pairs
def check_basic_ops(id_agent, agent, id_robot, robot):  # @UnusedVariable
    if not isinstance(agent, ExploringAgent):
        print('%s not exploring agent: %s' % (id_agent, describe_type(agent)))
        return Skipped('agent not ExploringAgent')
    

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

        try:
            for logs_format in formats:
                ds.set_log_format(logs_format)
                simulate_some(2)
                simulate_some(2)
        except UnsupportedSpec as e:
            logger.warn(e)
            return Skipped('UnsupportedSpec')
        
        assert not log_index.has_streams_for_robot(id_robot)
        log_index.reindex()
        assert log_index.has_streams_for_robot(id_robot)

        def expect_num(found, num, msg=""):
            if num == found:
                return
            msg += '-- Expected %d logs/streams, found %d. ' % (num, found)
            msg += '\n' + log_index.debug_summary(ignore_cache=False)
            raise Exception(msg)

        expect_num(len(log_index.get_streams_for_robot(id_robot)),
                        2 * len(formats))
        expect_num(len(log_index.get_streams_for(id_robot, id_agent)),
                        2 * len(formats),
                    "Looking for %r/%r" % (id_robot, id_agent))

        log_index.get_robot_spec(id_robot)

        expect_num(len(log_index.get_episodes_for_robot(id_robot)),
                        4 * len(formats))
        expect_num(len(log_index.get_episodes_for_robot(id_robot,
                                                              id_agent)),
                        4 * len(formats))

        learn_log(data_central, id_robot=id_robot, id_agent=id_agent)

        parts_skipped = []

        if isinstance(agent, ServoingAgent):
            try:
                task_servo(data_central, id_agent, id_robot,
                           max_episode_len=1,
                           num_episodes=1,
                           displacement=1,
                           id_episodes=None,
                           cumulative=False,
                           interval_print=None,
                           num_episodes_with_robot_state=0)
            except NotImplementedError as e:
                logger.warn(e)
                parts_skipped.append('servo')
        else:
            msg = 'Agent does not implement servo'
            logger.warn(msg)
            parts_skipped.append('servo')

        if isinstance(agent, PredictingAgent):
            try:       
                task_predict(data_central,
                     id_agent=id_agent,
                     id_robot=id_robot)
            except NotImplementedError as e:
                logger.warn(e)
                parts_skipped.append('predict')
        else:
            msg = 'Agent does not implement predictor'
            logger.warn(msg)
            parts_skipped.append('predict')
    
        if parts_skipped:
            return PartiallySkipped(parts_skipped)

        print('All tests completed')