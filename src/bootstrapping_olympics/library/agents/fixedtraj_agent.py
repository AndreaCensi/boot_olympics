'''A simple agent useful for testing.'''

import numpy as np
from contracts import contract
from bootstrapping_olympics import ExploringAgent

__all__ = ['FixedTrajectoryAgent']


class FixedTrajectoryAgent(ExploringAgent):
    '''
        An agent that executes a fixed pattern of commands for each episode.
                
    '''

    def __init__(self, plan):
        self.fixedtraj = FixedTrajectory()
        self.fixedtraj.add_plan(plan)

    def init(self, boot_spec):
        self.default_cmd = boot_spec.get_commands().get_default_value()

    def process_observations(self, observations):
        self.timestamp = observations['timestamp']

    def choose_commands(self):
        cmd = self.fixedtraj.choose_commands(self.timestamp)
        if cmd is None:
            return self.default_cmd
        else:
            value = np.array(cmd)
            return value

    def __repr__(self):
        return "FixedTrajectoryAgent()"


class FixedTrajectory(object):
    
    def __init__(self):
        self.pairs = []
        # last time we switched command
        self.last_timestamp = None
    
    @contract(steps='list[>=1](seq[2])')
    def add_plan(self, steps):
        for t in steps:
            duration, command = t
            self.add(duration, command)
            
    @contract(duration='>0', command='array')
    def add(self, duration, command):
        self.pairs.append((duration, command))
        
    def choose_commands(self, timestamp):
        # if we already have sent one command
        if self.last_timestamp is not None:
            # check for how long
            duration = timestamp - self.last_timestamp
            expected = self.steps[0][0]
            if duration > expected:
                # go to the next command
                self.steps.pop()
                self.last_timestamp = timestamp
        else:
            self.last_timestamp = timestamp  

        if not self.steps:  # finished
            return None
        
        return self.steps[0][1]
        
        
        
        
