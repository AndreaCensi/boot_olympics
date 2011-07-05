from ..interfaces import AgentInterface
import numpy as np

class RandomAgent(AgentInterface):
    
    def perform_task(self, task):
        return False
        
    def init(self, sensels_shape, commands_spec):
        self.num_commands = len(commands_spec)
    
    def process_observations(self, observations):
        pass
    
    def choose_commands(self):
        return np.random.rand(self.num_commands)    

    
