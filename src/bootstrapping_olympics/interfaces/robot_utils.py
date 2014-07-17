from .robot import BasicRobot
from blocks import NotReady
from blocks.library import WithQueue
from bootstrapping_olympics import RobotObservations
from bootstrapping_olympics.interfaces.observations import ObsKeeper
from contracts import contract

__all__ = [
    'RobotAsBlackBox3',
]

class RobotAsBlackBox3(WithQueue):
    """ This one does not look at handling NotReady correctly. 
    
        One observations is put in the queue when reset() is claled.
        Then every time we got new_commands using put, we put new observations.
    """

    # TODO: remove in favor of RobotAsBlackBox
    @contract(robot=BasicRobot)
    def __init__(self, robot):
        self.log_add_child('robot', robot)

        self.robot = robot
        self.last_obs = None
        self.id_robot = 'unnamed_robot'
        self.id_episode = None

    def reset(self):
        WithQueue.reset(self)
        self.keeper = ObsKeeper(boot_spec=self.robot.get_spec(),
                                id_robot=self.id_robot,
                                check_valid_values=False)

        self.episode = self.robot.new_episode()
        self.ended = False
        self._enqueue()

    def _enqueue(self):
        try:
            obs = self.robot.get_observations()
        except RobotObservations.NotReady as e:
            msg = 'Sorry, RobotAsBlackBox3 does not handle NotReady.'
            msg += '\n Not ready: %s' % e
            raise NotReady(msg)

        if self.last_obs is not None:
            if self.last_obs.timestamp == obs.timestamp:
                raise NotReady('got same obs')

        self.last_obs = obs

        if self.id_episode is not None:
            id_episode = self.id_episode
        else:
            id_episode = self.episode.id_episode

        bd = self.keeper.push(timestamp=obs.timestamp,
                             observations=obs.observations,
                             commands=obs.commands,
                             commands_source=obs.commands_source,
                             id_episode=id_episode,
                             id_world=self.episode.id_environment)

        self.append((obs.timestamp, bd))

        self.ended = obs.episode_end
    
        if self.ended:
            self.end_input()
            
    def __repr__(self):
        return 'RobotAsBlackBox(%r)' % self.robot

    @contract(value='tuple(float,*)')
    def put_noblock(self, value):
        if self.ended:
            msg = 'Calling put() after episode ended.'
            raise ValueError(msg)
            
        _, cmds = value
        self.robot.set_commands(*cmds)
        self._enqueue()
