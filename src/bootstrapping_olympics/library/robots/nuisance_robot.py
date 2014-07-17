from blocks import SimpleBlackBox, Source
from bootstrapping_olympics import (EpisodeDesc, RobotInterface, 
    get_conftools_robots)
from contracts import contract
from streamels.boot_spec import BootSpec


__all__ = [
    'NuisanceRobot',
]


class NuisanceRobot(RobotInterface):
    """ An agent that sees the data filtered through the given nuisances."""
    
    @contract(nuisances='list(str|code_spec|isinstance(RepresentationNuisanceCausal)'
                        '|isinstance(RepresentationNuisance))',
              robot='str|code_spec|isinstance(Basicrobot)')
    def __init__(self, nuisances, robot):
        from bootstrapping_olympics.library.agents.nuisance_agent_actions \
            import instance_nuisance_series
        self.nuisance = instance_nuisance_series(nuisances)
        
        config_robots = get_conftools_robots()
        _, self.robot = config_robots.instance_smarter(robot)
    
    @contract(returns=BootSpec)
    def get_spec(self):
        spec = self.robot.get_spec()
        spec2 = self.nuisance.transform_spec(spec)
        return spec2


    # exploration
    @contract(returns=EpisodeDesc)
    def new_episode(self):
        return self.robot.new_episode()
    
    @contract(returns=SimpleBlackBox)
    def get_active_stream(self):
        stream = self.robot.get_active_stream()
        from bootstrapping_olympics.library.agents.nuisance_agent_actions \
            import wrap_robot_exploration
        return wrap_robot_exploration(stream, self.nuisance)
    
#     @contract(returns=Source)
#     def get_passive_stream(self): 
#         source = self.robot.get_passive_stream()
#         from bootstrapping_olympics.library.agents.nuisance_agent_actions \
#             import wrap_robot_passive_stream
#         return wrap_robot_passive_stream(source, self.nuisance)
#     
    # deprecated    
    def get_observations(self):
        raise Exception('Using old API with set_commands().')
    
    # deprecated        
    def set_commands(self, commands, commands_source):  # @UnusedVariable
        raise Exception('Using old API with set_commands().')

    @contract(returns='None|dict')
    def get_state(self):
        return self.robot.get_state() 
    
    @contract(returns='list[>=1]')
    def get_inner_components(self):
        return [self] + self.robot.get_inner_components()
    
    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        raise NotImplementedError('to implement')
        
        
   
    