from bootstrapping_olympics import (RobotInterface, RobotObservations,
    get_boot_config)
from bootstrapping_olympics.interfaces.rep_nuisance_causal import (
    RepresentationNuisanceCausal)
from contracts import contract
from blocks.utils import WithQueue, bb_pump
from blocks.simple_black_box import SimpleBlackBox
from bootstrapping_olympics.interfaces.with_internal_log import BootWithInternalLog
import warnings

class RobotAsBlackBox(BootWithInternalLog, WithQueue):
    
    @contract(robot=RobotInterface)
    def __init__(self, robot):
        WithQueue.__init__(self)
        self.log_add_child('robot', robot)
        
        self.robot = robot
        self.last_obs = None

    def __repr__(self):
        return 'RobotAsBlackBox(%r)' % self.robot

    @contract(value='tuple(float,*)')
    def put(self, value):
        _, cmds = value
        self.robot.set_commands(*cmds)
        self._pump()
        
    def _pump(self):
        while True:
            try:
                obs = self.robot.get_observations()
            except RobotObservations.NotReady:
                break
    
            # Dont' give the same observations over and over again
            if self.last_obs is not None:
                if self.last_obs.timestamp == obs.timestamp:
                    break
                
            self.last_obs = obs
            
            cmd = obs.commands
            cmd_source = obs.commands_source
            self.queue.append((obs.timestamp, ((cmd, cmd_source), obs)))
    
    def get(self, block=True, timeout=None):  # @UnusedVariable
        self._pump()
        if not self.queue:
            raise SimpleBlackBox.NotReady()
        return self.queue.pop(0)
    

class EquivRobotCausal(RobotInterface):
     
    @staticmethod
    def from_yaml(robot, nuisance):
        boot_config = get_boot_config()
        _, robot = boot_config.robots.instance_smarter(robot)
        _, nuisance = boot_config.nuisances_causal.instance_smarter(nuisance)
        return EquivRobotCausal(robot=robot, nuisance=nuisance)
        
    @contract(robot=RobotInterface, nuisance=RepresentationNuisanceCausal)
    def __init__(self, robot, nuisance):
        self.robot = robot
        self.nuisance = nuisance
        
        self.spec1 = self.robot.get_spec()
        self.spec2 = self.nuisance.transform_spec(self.spec1)
         
        self.pre = self.nuisance.get_pre()
        self.robotw = RobotAsBlackBox(self.robot)
        self.post = self.nuisance.get_post()

        self.log_add_child('pre', self.pre)
        self.log_add_child('post', self.post)
        self.log_add_child('robotw', self.robotw)
        
        self.last_obs = None
        self.last_commands = None
    
    def get_spec(self):
        return self.spec2
    
    def set_commands(self, commands, commands_source):
        # need to get timestamp
        if self.last_obs is None:
            self.last_obs = self.robot.get_observations()
        t = self.last_obs.timestamp
        self.info('EquivRobotCausal:set_commands()')
        self.last_commands = (commands, commands_source)
        self.pre.put((t, self.last_commands))
        bb_pump(self.pre, self.robotw)
          
    def get_observations(self):
        self.info('EquivRobotCausal:get_observations()')
        bb_pump(self.robotw, self.post)
        try:
            _, obs1 = self.post.get(block=False)
            if self.last_commands is None:
                warnings.warn('Should get default value from spec otherwise fail')
                pass
            else:
                obs1.commands, obs1.commands_source = self.last_commands
            return obs1
        except SimpleBlackBox.NotReady:
            raise RobotObservations.NotReady()
        
    def new_episode(self):
        return self.robot.new_episode()

    def get_inner_components(self):
        return [self] + self.robot.get_inner_components()
            

