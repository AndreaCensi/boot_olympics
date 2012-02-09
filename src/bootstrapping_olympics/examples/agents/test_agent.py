'''A simple agent useful for testing.'''

from ... import AgentInterface
import numpy as np

__all__ = ['TestAgent']


class TestAgent(AgentInterface):
    ''' 
        A simple agent useful for testing.
    
        It executes any command specified on the constructor. 
        If the command is a string, then it is evaluated. 
        You can use the variable "t" for time.        
    '''

    def __init__(self, cmd):
        self.cmd = cmd

    def init(self, boot_spec):
        self.default_cmd = boot_spec.get_commands().get_default_value()

    def process_observations(self, observations):
        self.timestamp = observations['timestamp']

    def choose_commands(self):
        if isinstance(self.cmd, str):
            t = self.timestamp #@UnusedVariable
            cmd = eval(self.cmd)
            value = np.array(cmd)
        else:
            value = np.array(self.cmd)

        if value.shape != self.default_cmd.shape:
            #self.info('Invalid shape; patching up.')
            n = min(value.size, self.default_cmd.size)
            self.default_cmd.flat[:n] = value.flat[:n]
            return self.default_cmd
        else:
            return value

    def __repr__(self):
        return "TestAgent"
