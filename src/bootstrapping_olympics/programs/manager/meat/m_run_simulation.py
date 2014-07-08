import time
import warnings

from contracts import contract

from blocks import NotReady, Finished
from blocks import SimpleBlackBox
from blocks.composition import series
from blocks.library import Info
from blocks.pumps import bb_get_block_poll_sleep
from bootstrapping_olympics import (RobotInterface, RobotObservations,
    AgentInterface, ObsKeeper, logger)


__all__ = ['run_simulation']

@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, agent=AgentInterface, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, id_agent, agent, max_observations,
                   max_time,
                   check_valid_values=True, id_episode=None):
    ''' 
        Runs an episode of the simulation. The agent should already been
        init()ed. 
    '''
    logger.info('run_simulation(max_time=%s; max_observations=%s)'
                % (max_time, max_observations))
#     episode = robot.new_episode()
    
    robot_sys = RobotAsBlackBox2(id_robot, robot)

    warnings.warn('we are not honoring id_episode')

    logger.info('max_observations: %s' % max_observations)
    logger.info('max_time: %s' % max_time)

    from bootstrapping_olympics.library.agents.nuisance_agent_actions import CheckBootSpec
    cmd_spec = robot.get_spec().get_commands()
#     robot_sys = series(robot_sys,  # CheckBootSpec(robot.get_spec()),
#                        Info())
#     robot_sys.set_names(['robot_sys', 'Info'])
    robot_sys.set_name_for_log('robot_sys')
    robot_sys.reset()  # = new_episode
    

    agent_sys = agent.get_explorer()
    agent_sys.set_name_for_log('agent_sys')
    agent_sys.reset()

    counter = 0
    while counter < max_observations:
        logger.info('looop %d' % counter)
        try:
            t, bd = robot_sys.get(block=True)
        except NotReady:
            raise Exception('Hey, cannot obtain NotReady when block is True')
            warnings.warn('remove')
            logger.info('not ready')
            time.sleep(0.001)
            continue
        except Finished:
            logger.info('Episode ended at %s due to obs.episode_end.'
                         % counter)
            break
        
        if bd['time_from_episode_start'] > max_time:
            logger.info('Episode ended at %s for time limit %s > %s ' % 
                         (counter, bd['time_from_episode_start'],
                          max_time))
            break

        yield bd

        logger.info('putting into agent')
        agent_sys.put((t, bd), block=True)

        logger.info('getting commands')
        print agent_sys
        commands = agent_sys.get(block=True)

        if check_valid_values:
            cmd_spec.check_valid_value(commands)

        logger.info('putting into robots')
        timestamp = t
        robot_sys.put((timestamp, (commands, id_agent)), block=True)

        counter += 1
    
    
class RobotAsBlackBox2(SimpleBlackBox):

    # TODO: remove in favor of RobotAsBlackBox
    @contract(robot=RobotInterface)
    def __init__(self, id_robot, robot, sleep=0.1):
        self.log_add_child('robot', robot)

        self.sleep = sleep
        self.robot = robot
        self.last_obs = None
        self.id_robot = id_robot

    def reset(self):
        self.info('resetting')
        self.keeper = ObsKeeper(boot_spec=self.robot.get_spec(),
                                id_robot=self.id_robot,
                                check_valid_values=False)

        self.episode = self.robot.new_episode()

    def __repr__(self):
        return 'RobotAsBlackBox(%r)' % self.robot

    @contract(value='tuple(float,*)')
    def put(self, value, block=True, timeout=None):
        _, cmds = value
        self.robot.set_commands(*cmds)

    def get(self, block=True, timeout=None):
        if block:
            res = bb_get_block_poll_sleep(self,
                                           timeout=timeout,
                                           sleep=self.sleep)
            self.info('returning from blocking get(): %s' % str(res))
            return res
        else:
            return self._get_notblock()

    def _get_notblock(self):
        self.info('getting')
        try:
            obs = self.robot.get_observations()
        except RobotObservations.NotReady:
            raise NotReady()

        if self.last_obs is not None:
            if self.last_obs.timestamp == obs.timestamp:
                raise NotReady()

        self.last_obs = obs

        boot_observations = self.keeper.push(timestamp=obs.timestamp,
                                             observations=obs.observations,
                                             commands=obs.commands,
                                             commands_source=obs.commands_source,
                                             id_episode=self.episode.id_episode,
                                             id_world=self.episode.id_environment)
        res = boot_observations

        self.info('returning %s' % res)

        return obs.timestamp, res

    
    
    
    
    


