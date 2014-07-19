from blocks import SimpleBlackBox, SimpleBlackBoxTN
from bootstrapping_olympics import BootSpec, Constants, EpisodeDesc
from bootstrapping_olympics import BasicRobot, ExplorableRobot
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
import time
from blocks.library import WithQueue
from blocks.library.timed.checks import check_timed_named

__all__ = ['RandomRobot']


class RandomRobot(BasicRobot, ExplorableRobot):
    ''' 
        This "Robot" generates uniform noise,
        and ignores any command given.
        
        You can have a fixed robot by setting ``y0`` in the constructor.
    '''
    
    @contract(boot_spec='dict|BootSpec', dt='float,>0', t0='None|float')
    def __init__(self, boot_spec,
                 dt=Constants.DEFAULT_SIMULATION_DT, t0=None, y0=None):
        if t0 is None:
            t0 = time.time()
        self.timestamp = t0
        self.dt = dt
        if not isinstance(boot_spec, BootSpec):
            boot_spec = BootSpec.from_yaml(boot_spec) 
        self.spec = boot_spec
        self.commands = self.spec.get_commands().get_default_value()
        self.commands_source = Constants.CMD_SOURCE_REST
        self.y0 = y0

    def __repr__(self):
        return 'RandomRobot(%r)' % self.spec
        
    def get_spec(self):
        return self.spec 

    @contract(returns=SimpleBlackBox)
    def get_active_stream(self):
        
        class RandomRobotStream(WithQueue, SimpleBlackBoxTN):
            def __init__(self, random_robot):
                WithQueue.__init__(self)
                self.random_robot = random_robot
                
            def reset(self):
                WithQueue.reset(self)
                self.episode = self.random_robot.new_episode()
                self.ended = False
                     
            @contract(value='tuple(float,*)')
            def put_noblock(self, value):
                check_timed_named(value, self)
                
                (timestamp, (sname, _)) = value
                if not sname in ['commands']:
                    msg = 'Unexpected signal %r.' % sname
                    raise ValueError(msg)
                
                if self.random_robot.y0 is None:
                    obs = self.random_robot.spec.get_observations().get_random_value()
                else:
                    obs = self.y0
                    
                x = (timestamp + self.random_robot.dt, ('observations', obs))
                self.append(x)
                
        return RandomRobotStream(self)
                
                 
    def __str__(self):
        return 'RandomRobot(%s;%s)' % (self.spec.get_observations(),
                                       self.spec.get_commands())

    def new_episode(self):
        self.timestamp = time.time()
        return EpisodeDesc(unique_timestamp_string(),
                           'n/a')  # XXX: add constant




