from . import contract
from bootstrapping_olympics import (Constants, RobotObservations, BootSpec,
    RobotInterface, EpisodeDesc)
from bootstrapping_olympics.utils import unique_timestamp_string
import time

__all__ = ['RandomRobot']


class RandomRobot(RobotInterface):
    ''' This is a sensorimotor cascade generating uniform noise,
        and ignores any command given. '''

    def __init__(self, boot_spec, dt=Constants.DEFAULT_SIMULATION_DT):
        self.timestamp = time.time()
        self.dt = dt
        self.spec = BootSpec.from_yaml(boot_spec)
        self.commands = self.spec.get_commands().get_default_value()
        self.commands_source = Constants.CMD_SOURCE_REST

    def get_spec(self):
        return self.spec

    def get_observations(self):
        obs = self.spec.get_observations().get_random_value()
        return RobotObservations(timestamp=self.timestamp,
                                 observations=obs,
                                 commands=self.commands,
                                 commands_source=self.commands_source,
                                 episode_end=False,
                                 robot_pose=None)

    def __str__(self):
        return 'RandomRobot(%s;%s)' % (self.spec.get_observations(),
                                       self.spec.get_commands())

    @contract(commands='array')
    def set_commands(self, commands, commands_source):
        self.timestamp += self.dt
        self.commands = commands
        self.commands_source = commands_source

    def new_episode(self):
        self.timestamp = time.time()
        return EpisodeDesc(unique_timestamp_string(),
                           'n/a')  # XXX: add constant

