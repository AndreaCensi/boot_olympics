from .. import RobotObservations, BootSpec, RobotInterface, EpisodeDesc
from ..utils import isodate_with_secs
from contracts import contract

__all__ = ['RandomRobot']

class RandomRobot(RobotInterface):
    ''' This is a sensorimotor cascade generating uniform noise,
        and ignores any command given. '''
    
    def __init__(self, boot_spec, dt=0.1):
        self.timestamp = 0
        self.dt = dt
        self.spec = BootSpec.from_yaml(boot_spec)
        self.commands = self.spec.get_commands().get_default_value()
        self.commands_source = 'rest' # XXX: add constant
        
    def get_spec(self):
        return self.spec 
        
    def get_observations(self):
        obs = self.spec.get_observations().get_random_value()
        return RobotObservations(timestamp=self.timestamp,
                                 observations=obs,
                                 commands=self.commands,
                                 commands_source=self.commands_source)    

    @contract(commands='array')
    def set_commands(self, commands):
        self.timestamp += self.dt
        self.commands = commands
        self.commands_source = 'agent'  # XXX: add constant
    
    def new_episode(self):
        self.timestamp = 0.0
        return EpisodeDesc(isodate_with_secs(), 'n/a')  # XXX: add constant
        
