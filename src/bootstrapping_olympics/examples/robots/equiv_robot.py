from .. import contract
from ... import BootSpec, RobotInterface, BootOlympicsConfig
from bootstrapping_olympics.interfaces.stream_spec import StreamSpec

__all__ = ['EquivRobot']


class EquivRobot(RobotInterface):
    ''' This is used to create the perturbed version of a robot. '''
    
    @contract(robot='str',
              obs_nuisance='str|list(str)',
              cmd_nuisance='str|list(str)')
    def __init__(self, robot, obs_nuisance=[], cmd_nuisance=[]):
        self.robot = BootOlympicsConfig.robots.instance(robot) #@UndefinedVariable
        
        # convert to (possibly empty) list of strings
        if isinstance(obs_nuisance, str): obs_nuisance = [obs_nuisance]
        if isinstance(cmd_nuisance, str): cmd_nuisance = [cmd_nuisance]
        
        self.obs_nuisances = [BootOlympicsConfig.nuisances.instance(x) #@UndefinedVariable
                         for x in obs_nuisance]     
        self.cmd_nuisances = [BootOlympicsConfig.nuisances.instance(x) #@UndefinedVariable
                         for x in cmd_nuisance]     
        self.cmd_nuisances_inv = [x.inverse() for x in self.cmd_nuisances]     
        
        obs_spec = self.robot.get_spec().get_observations()
        for n in self.obs_nuisances:
            obs_spec = n.transform_spec(obs_spec)
        
        cmd_spec = self.robot.get_spec().get_commands()
        for n in self.cmd_nuisances:
            cmd_spec = n.transform_spec(cmd_spec)
        
        # now initialize in reverse
        cmd_spec_i = cmd_spec
        for n in reversed(self.cmd_nuisances_inv):
            cmd_spec_i = n.transform_spec(cmd_spec_i)
            
        StreamSpec.check_same_spec(cmd_spec_i,
                                   self.robot.get_spec().get_commands())
        
        self.spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec)

    def get_spec(self):
        return self.spec 
        
    def get_observations(self):
        obs = self.robot.get_observations()
        values = obs.observations
        for n in self.obs_nuisances:
            values = n.transform_value(values)
        obs.observations = values
        
        # Note we use the inverted order
        values = obs.commands
        for n in reversed(self.cmd_nuisances_inv):
            values = n.transform_value(values)
        obs.commands = values
        
        return obs

    @contract(commands='array')
    def set_commands(self, commands, commands_source):
        for n in self.cmd_nuisances:
            commands = n.transform_value(commands)
        self.robot.set_commands(commands, commands_source)
        
    def new_episode(self):
        return self.robot.new_episode()

