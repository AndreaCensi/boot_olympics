from bootstrapping_olympics import BootSpec, RobotInterface
from bootstrapping_olympics.utils import isodate
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
        RobotInterface.__init__(self)
        
    def get_spec(self):
        return BootSpec.from_yaml({
            'commands': {
                'shape': [self.num_commands],
                'format': 'C',
                'range': [-1, 1]
            },
            'observations': {
                'shape': [self.num_sensels],
                'format': 'C',
                'range': [0, 1]
            }        
        })
        
    def get_observations(self):
        return self.timestamp, np.random.rand(self.num_sensels)    

    def set_commands(self, dt):
        self.timestamp += dt
    
    def new_episode(self):
        self.id_episode = isodate()
        self.id_environment = 'n/a'
        self.timestamp = 0
        
