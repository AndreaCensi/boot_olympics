from .. import RobotObservations, BootSpec, RobotInterface, EpisodeDesc
from ..utils import isodate
import numpy as np

__all__ = ['RandomRobot']

class RandomRobot(RobotInterface):
    ''' This is a sensorimotor cascade generating Gaussian noise,
        and ignores any command given. '''
    
    def __init__(self, num_sensels=1, num_commands=1, dt=0.1):
        self.num_sensels = num_sensels
        self.num_commands = num_commands
        self.timestamp = 0
        self.dt = dt
        self.commands = np.zeros(self.num_commands)
        self.commands_source = 'rest'
        
    def get_spec(self):
        return BootSpec.from_yaml({
            'commands': {
                'shape': [self.num_commands],
                'format': 'C',
                'range': [-1, 1],
                'rest': [0] * self.num_commands
            },
            'observations': {
                'shape': [self.num_sensels],
                'format': 'C',
                'range': [0, 1]
            }        
        })
        
    def get_observations(self):
        obs = np.random.rand(self.num_sensels)
        return RobotObservations(self.timestamp, obs, commands=self.commands,
                                 commands_source=self.commands_source)    

    def set_commands(self, commands):
        self.timestamp += self.dt
        self.commands = commands
        self.commands_source = 'agent'
    
    def new_episode(self):
        self.timestamp = 0
        return EpisodeDesc(isodate(), 'n/a')
        
