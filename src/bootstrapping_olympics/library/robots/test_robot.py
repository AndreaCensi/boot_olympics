'''A simple robot useful for testing.'''

from blocks import SimpleBlackBoxTN
from blocks.library.simple import WithQueue
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics import (
    BasicRobot, BootSpec, Constants, EpisodeDesc, ExplorableRobot)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
import numpy as np
import time


__all__ = ['TestRobot']


class TestRobot(BasicRobot, ExplorableRobot):
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
        self.dt = dt
        self.value = value
        self.spec = BootSpec.from_yaml(boot_spec)

    @contract(returns='array')
    def compute_observations(self, t, u):  # @UnusedVariable
        """ Computes the observations from the user string. """
        value = eval(self.value)
        value = np.array(value)
        return value

    def get_spec(self):
        return self.spec

    def get_active_stream(self):
          
        class TestRobotStream(WithQueue, SimpleBlackBoxTN):
            def __init__(self, robot):
                WithQueue.__init__(self)
                self.robot = robot
                
            def reset(self):
                WithQueue.reset(self)
                episode = self.robot.new_episode()  # @UnusedVariable
                
                self.t0 = time.time()
                self.timestamp = self.t0
                self.last_u = None
                # enqueue one observation
                self._enqueue()
                     
            def _enqueue(self):
                u = self.last_u
                self.timestamp += self.robot.dt
                t = self.timestamp-self.t0
                obs = self.robot.compute_observations(t=t, u=u)
                x = (self.timestamp, ('observations', obs))
                self.append(x)
                
            @contract(value='tuple(float,*)')
            def put_noblock(self, value):
                check_timed_named(value, self)
                
                (timestamp, (sname, x)) = value
                
                if not sname in ['commands']:
                    msg = 'Unexpected signal %r.' % sname
                    raise ValueError(msg)
                 
                if sname == 'commands':
                    if timestamp <  self.timestamp:
                        msg = 'Commands with invalid timestamp %.5f < %.5f' %(timestamp, self.timestamp)
                        raise ValueError(msg)
                    
                    self.last_u = x
                    self._enqueue()
                
        return TestRobotStream(self)

    def __str__(self):
        return 'TestRobot'

    def set_commands(self, commands, commands_source):
        self.timestamp += self.dt
        self.commands = commands
        self.commands_source = commands_source

    def new_episode(self):
        self.timestamp = time.time()
        episode = EpisodeDesc(unique_timestamp_string(), 'n/a')
        # print('Creating episode %s' % episode)
        return episode

