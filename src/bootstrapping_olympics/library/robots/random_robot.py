from blocks import (SimpleBlackBox, SimpleBlackBoxTN, WithQueue, 
    check_timed_named)
from bootstrapping_olympics import (
    BasicRobot, BootSpec, Constants, EpisodeDesc, ExplorableRobot)
from bootstrapping_olympics.utils import unique_timestamp_string
from contracts import contract
from geometry import SE2_from_rotation_translation, SE3_from_SE2
import numpy  as np
import time

__all__ = ['RandomRobot']


class RandomRobot(BasicRobot, ExplorableRobot):
    ''' 
        This "Robot" generates uniform noise,
        and ignores any command given.
        
        You can have a fixed robot by setting ``y0`` in the constructor.
    '''
    
    @contract(boot_spec='dict|BootSpec', dt='float,>0')
    def __init__(self, boot_spec,
                 dt=Constants.DEFAULT_SIMULATION_DT, y0=None,
                 use_robot_pose=False):
        self.dt = dt
        if not isinstance(boot_spec, BootSpec):
            boot_spec = BootSpec.from_yaml(boot_spec) 
        self.spec = boot_spec
        
        self.y0 = y0
        self.use_robot_pose = use_robot_pose
 

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
                self.random_robot.timestamp = time.time()
                self.ended = False
                
                self._enqueue()
                
            def _enqueue(self):
                """ Robot must provide a first observations. """
                if self.random_robot.y0 is None:
                    obs = self.random_robot.spec.get_observations().get_random_value()
                else:
                    obs = self.y0            
                t = self.random_robot.timestamp
                self.append((t, ('observations', obs)))
                # Note that if we give a fixed one then we converge
                # right away from servoing
                #    robot_pose = np.eye(4)
                if self.random_robot.use_robot_pose:
                    tran = np.random.rand(2) * 10
                    robot_pose = SE3_from_SE2(SE2_from_rotation_translation(np.eye(2), tran))
                    self.append((t, ('robot_pose', robot_pose)))
                    
            @contract(value='tuple(float,*)')
            def put_noblock(self, value):
                check_timed_named(value, self)
                (timestamp, (sname, _)) = value
                
                #self.info('Seen %s %s' % (timestamp, sname))
                if not sname in ['commands']:
                    msg = 'Unexpected signal %r.' % sname
                    raise ValueError(msg)
             
                if timestamp < self.random_robot.timestamp :
                    msg = ('Commands received at %.5f but current timestamp is %.5f. ' 
                           % (timestamp, self.random_robot.timestamp))
                    raise ValueError(msg)
                     
                self.random_robot.timestamp += self.random_robot.dt
                
                self._enqueue()
                
        return RandomRobotStream(self)
                
                 
    def __str__(self):
        return 'RandomRobot(%s;%s)' % (self.spec.get_observations(),
                                       self.spec.get_commands())

    def new_episode(self):
        #self.timestamp = time.time()
        return EpisodeDesc(unique_timestamp_string(), 'n/a')  # XXX: add constant




