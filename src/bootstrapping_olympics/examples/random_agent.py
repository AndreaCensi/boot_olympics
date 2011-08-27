from ..interfaces import AgentInterface

from ..interfaces.commands_utils import random_commands

__all__ = ['RandomAgent']

class RandomAgent(AgentInterface):
    
    def perform_task(self, task):
        return False
        
    def init(self, sensels_shape, commands_spec):
        self.commands_spec = commands_spec
    
    def process_observations(self, observations):
        pass
    
    def choose_commands(self):
        return random_commands(commands_spec=self.commands_spec)
    
    def __repr__(self):
        return "RandomAgent"
    
    
