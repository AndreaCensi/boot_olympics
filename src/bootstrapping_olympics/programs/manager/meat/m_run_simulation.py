from blocks import NeedInput, NotReady
from blocks.composition import series
from blocks.exceptions import Finished
from blocks.library import CheckSequence
from bootstrapping_olympics import (ExplorableRobot, ExploringAgent, 
    RobotInterface, logger)
from contracts import contract
from contracts.utils import check_isinstance
import warnings


__all__ = ['run_simulation']

@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, agent=ExploringAgent, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, id_agent, agent, max_observations,
                   max_time,
                   check_valid_values=True):
    ''' 
        Runs one episode of the simulation until it ends. 
        
        The agent should already been init()ed. 
        
        Yields the bd.
    '''
#     
#     if id_episode is None:
#         msg = 'In the new system you have to pass explicit id_episode'
#         raise Exception(msg)
#     
    check_isinstance(agent, ExploringAgent)
    check_isinstance(robot, ExplorableRobot)

    warnings.warn('we are not honoring id_episode')


    logger.info('run_simulation(max_time=%s; max_observations=%s)'
                % (max_time, max_observations))

    
    robot_as_sys = robot.get_active_stream()
    
    
    logger.info('max_observations: %s' % max_observations)
    logger.info('max_time: %s' % max_time)

    cmd_spec = robot.get_spec().get_commands()

    if True:
        robot_sys = series(robot_as_sys,  # CheckBootSpec(robot.get_spec()),
                            CheckSequence())

        robot_sys.set_names(['robot_sys', 'CheckSeq'])
        robot_sys.set_name_for_log('robot_sys')
    else:
        robot_sys = robot_as_sys
    robot_sys.reset()  # = new_episode
    

    agent_sys = agent.get_explorer()
    agent_sys.set_name_for_log('agent_sys')
    agent_sys.reset()

    counter = 0
    t0 = None
    while counter < max_observations:
        try:
            t, bd = robot_sys.get(block=True)
            if t0 is None:
                t0 = t
        except NotReady:
            raise Exception('Hey, cannot obtain NotReady when block is True')
        except Finished:
            logger.info('Episode ended at %s due to obs.episode_end.'
                         % counter)
            break
            
#         bd['id_robot'] = id_robot
#         if id_episode is not None:
#         bd['id_episode'] = id_episode
            
        time_from_episode_start = t-t0
        if time_from_episode_start > max_time:
            logger.info('Episode ended at %s for time limit %s > %s ' % 
                         (counter, time_from_episode_start, max_time))
            break

        yield t, bd

        # logger.info('putting into agent')
        agent_sys.put((t, bd), block=True)

        # logger.info('getting commands')
        try:
            commands = agent_sys.get(block=True)
        except NeedInput:
            if counter > 3:
                msg = 'Agent raises NeedInput after %s steps.' % counter
                raise Exception(msg)
            else:
                logger.warn('after %d steps, using default commands because Agent not ready.' % counter)
            commands = cmd_spec.get_default_value()

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        # logger.info('putting into robots')
        timestamp = t
        robot_sys.put((timestamp, (commands, id_agent)), block=True)

        counter += 1



#
#
# class RobotAsBlackBox2(SimpleBlackBox):
#
#     # TODO: remove in favor of RobotAsBlackBox
#     @contract(robot=RobotInterface)
#     def __init__(self, id_robot, robot, sleep=0.1):
#         self.log_add_child('robot', robot)
#
#         self.sleep = sleep
#         self.robot = robot
#         self.last_obs = None
#         self.id_robot = id_robot
#
#     def reset(self):
#         self.info('resetting')
#         self.keeper = ObsKeeper(boot_spec=self.robot.get_spec(),
#                                 id_robot=self.id_robot,
#                                 check_valid_values=False)
#
#         self.episode = self.robot.new_episode()
#
#     def __repr__(self):
#         return 'RobotAsBlackBox(%r)' % self.robot
#
#     @contract(value='tuple(float,*)')
#     def put(self, value, block=True, timeout=None):
#         _, cmds = value
#         self.robot.set_commands(*cmds)
#
#     def get(self, block=True, timeout=None):
#         if block:
#             res = bb_get_block_poll_sleep(self,
#                                            timeout=timeout,
#                                            sleep=self.sleep)
# #             self.info('returning from blocking get(): %s' % str(res))
#             return res
#         else:
#             return self._get_notblock()
#
#     def _get_notblock(self):
#         self.info('getting')
#         try:
#             obs = self.robot.get_observations()
#         except RobotObservations.NotReady as e:
#             raise NotReady('robot not ready: %d' % e)
#
#         if self.last_obs is not None:
#             if self.last_obs.timestamp == obs.timestamp:
#                 raise NotReady('got same obs')
#
#         self.last_obs = obs
#
#         boot_observations = self.keeper.push(timestamp=obs.timestamp,
#                                              observations=obs.observations,
#                                              commands=obs.commands,
#                                              commands_source=obs.commands_source,
#                                              id_episode=self.episode.id_episode,
#                                              id_world=self.episode.id_environment)
#         res = boot_observations
#
# #         self.info('returning %s' % res)
#
#         return obs.timestamp, res
#
#
#

