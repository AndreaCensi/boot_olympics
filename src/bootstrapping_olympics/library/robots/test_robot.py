'''A simple robot useful for testing.'''

from . import contract, np
from bootstrapping_olympics import (BootSpec, Constants, RobotInterface,
    RobotObservations, EpisodeDesc)
from bootstrapping_olympics.utils import unique_timestamp_string
import time


__all__ = ['TestRobot']


class TestRobot(RobotInterface):
    '''
        A simple robot useful for testing.
    
        It implements any instantaneous relation
        between *y* and *u*.

        You can use the variables:
        
        - ``t`` for time.
        - ``u`` for the commands  
        - ``np`` for numpy reference      
    '''

    @contract(value='str', dt='>0')
    def __init__(self, boot_spec, value,
                 dt=Constants.DEFAULT_SIMULATION_DT):
        self.timestamp = time.time()
        self.dt = dt
        self.value = value
        self.spec = BootSpec.from_yaml(boot_spec)
        self.commands = self.spec.get_commands().get_default_value()
        self.commands_source = Constants.CMD_SOURCE_REST

    @contract(returns='array')
    def compute_observations(self):
        """ Computes the observations from the user string. """
        t = self.timestamp #@UnusedVariable
        u = self.commands #@UnusedVariable
        value = eval(self.value)
        value = np.array(value)
        #print('%s -> %s' % (self.value, value))
        return value

    def get_spec(self):
        return self.spec

    def get_observations(self):
        obs = self.compute_observations()
        return RobotObservations(timestamp=self.timestamp,
                                 observations=obs,
                                 commands=self.commands,
                                 commands_source=self.commands_source,
                                 episode_end=False,
                                 robot_pose=None)

    def __str__(self):
        return 'TestRobot'

    def set_commands(self, commands, commands_source):
        self.timestamp += self.dt
        self.commands = commands
        self.commands_source = commands_source

    def new_episode(self):
        self.timestamp = time.time()
        episode = EpisodeDesc(unique_timestamp_string(), 'n/a')
        #print('Creating episode %s' % episode)
        return episode

