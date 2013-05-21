'''A simple agent useful for testing.'''

from bootstrapping_olympics import AgentInterface
import numpy as np

__all__ = ['TestAgent']


class TestAgent(AgentInterface):
    '''
        A simple agent useful for testing.
    
        It executes any command specified on the constructor. 
        If the command is a string, then it is eval()uated.
         
        You can use the variables:
        
        - "t" for time.
        - "y" for the observations.        
    '''

    def __init__(self, cmd):
        self.cmd = cmd

    def init(self, boot_spec):
        self.default_cmd = boot_spec.get_commands().get_default_value()

    def process_observations(self, observations):
        self.timestamp = observations['timestamp']
        self.y = observations['observations']

    def choose_commands(self):
        if isinstance(self.cmd, str):
            # Variables that can be used in the expression
            y = self.y  # @UnusedVariable
            t = self.timestamp  # @UnusedVariable
            cmd = eval(self.cmd)
            value = np.array(cmd)
        else:
            value = np.array(self.cmd)

        if value.shape != self.default_cmd.shape:
            # self.info('Invalid shape; patching up.')
            n = min(value.size, self.default_cmd.size)
            self.default_cmd.flat[:n] = value.flat[:n]
            return self.default_cmd
        else:
            return value

    def __repr__(self):
        return "TestAgent"
