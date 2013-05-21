from contracts import contract
from bootstrapping_olympics import (Constants, RobotObservations, BootSpec,
    RobotInterface, EpisodeDesc)
from bootstrapping_olympics.utils import unique_timestamp_string
import time

__all__ = ['RandomRobot']


class RandomRobot(RobotInterface):
    ''' 
        This "Robot" generates uniform noise,
        and ignores any command given.
        
        You can have a fixed robot by setting ``y0`` in the constructor.
    '''
    
    @contract(boot_spec='dict|BootSpec', dt='float,>0', t0='None|float')
    def __init__(self, boot_spec,
                 dt=Constants.DEFAULT_SIMULATION_DT, t0=None, y0=None):
        if t0 is None:
            t0 = time.time()
        self.timestamp = t0
        self.dt = dt
        if not isinstance(boot_spec, BootSpec):
            boot_spec = BootSpec.from_yaml(boot_spec) 
        self.spec = boot_spec
        self.commands = self.spec.get_commands().get_default_value()
        self.commands_source = Constants.CMD_SOURCE_REST
        self.y0 = y0

    def __repr__(self):
        return 'RandomRobot(%r)' % self.spec
        
    def get_spec(self):
        return self.spec

    def get_observations(self):
        if self.y0 is None:
            obs = self.spec.get_observations().get_random_value()
        else:
            obs = self.y0
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
        print('set_commands %s at %s' % (commands, self.timestamp))
        self.commands = commands
        self.commands_source = commands_source
        self.timestamp += self.dt

    def new_episode(self):
        self.timestamp = time.time()
        return EpisodeDesc(unique_timestamp_string(),
                           'n/a')  # XXX: add constant




