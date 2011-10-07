from ..interfaces import AgentInterface

__all__ = ['RandomAgent']

class RandomAgent(AgentInterface):
    
    def init(self, boot_spec):
        self.commands_spec = boot_spec.get_commands()
    
    def process_observations(self, observations):
        pass
    
    def choose_commands(self):
        return self.commands_spec.get_random_value()
    
    def __repr__(self):
        return "RandomAgent"
    
    
