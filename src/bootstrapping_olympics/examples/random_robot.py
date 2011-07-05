from ..interfaces import RobotSimulationInterface
import numpy as np

class RandomRobot(RobotSimulationInterface):
    ''' This is a random robot generating Gaussian noise,
        and ignores any command given. '''
    
    def __init__(self, num_sensels=1, num_commands=1):
        self.num_sensels = num_sensels
        sensels_dtype = np.dtype(('float32', num_sensels))
        commands_spec = [(-1, +1)] * num_commands
        RobotSimulationInterface.__init__(self, sensels_dtype, commands_spec)
        
    def compute_observations(self):
        return np.random.randn(self.num_sensels)    

    def simulate_system(self, commands, dt):
        pass
