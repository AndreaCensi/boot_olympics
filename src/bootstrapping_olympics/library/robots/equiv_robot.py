from bootstrapping_olympics import get_conftools_nuisances, get_conftools_robots
from bootstrapping_olympics.library.nuisances_causal import (SimpleRNCCmd, 
    SimpleRNCObs)
from .nuisance_robot import NuisanceRobot
from contracts import contract
import warnings


__all__ = ['EquivRobot']

@contract(robot='str|dict|code_spec',
              obs_nuisance='str|list(str|dict|code_spec)',
              cmd_nuisance='str|list(str|dict|code_spec)')
def EquivRobot(robot, obs_nuisance=[], cmd_nuisance=[]):
    
    ''' This is used to create the perturbed version of a robot. '''
    _, robot = get_conftools_robots().instance_smarter(robot)
    

    if isinstance(obs_nuisance, str):
        obs_nuisance = [obs_nuisance]
    if isinstance(cmd_nuisance, str):
        cmd_nuisance = [cmd_nuisance]

    instance = lambda y: get_conftools_nuisances().instance_smarter(y)[1]
    
    obs_nuisances = [instance(x) for x in obs_nuisance]
    cmd_nuisances = [instance(x) for x in cmd_nuisance]


    # convert to causal
    obs_rnc = [SimpleRNCObs(x) for x in obs_nuisances]
    cmd_rnc = [SimpleRNCCmd(x) for x in cmd_nuisances]
    
    warnings.warn('Should think more about the order.')
    nuisances = obs_rnc + cmd_rnc
    return NuisanceRobot(nuisances=nuisances, robot=robot) 
            
#         
# 
# class EquivRobot(RobotInterface):
#     ''' This is used to create the perturbed version of a robot. '''
# 
#     @contract(robot='str|dict|code_spec',
#               obs_nuisance='str|list(str|dict|code_spec)',
#               cmd_nuisance='str|list(str|dict|code_spec)')
#     def __init__(self, robot, obs_nuisance=[], cmd_nuisance=[]):
#         self.inner_robot_name = robot 
# 
#         boot_config = get_boot_config()
#         id_robot, self.robot = boot_config.robots.instance_smarter(robot)
# 
#         if not isinstance(self.robot, RobotInterface):
#             msg = 'Expected RobotInterface, got %s' % describe_type(self.robot)
#             raise ValueError(msg)
#         
#         warnings.warn('handle the case better')
#         self.desc = ('EquivRobot(%s,obs:%s,cmd:%s)'
#                     % (id_robot, obs_nuisance, cmd_nuisance))
# 
#         # convert to (possibly empty) list of strings
#         if isinstance(obs_nuisance, str):
#             obs_nuisance = [obs_nuisance]
#         if isinstance(cmd_nuisance, str):
#             cmd_nuisance = [cmd_nuisance]
# 
#         instance = lambda y: boot_config.nuisances.instance_smarter(y)[1]
#         
#         self.obs_nuisances = [instance(x) for x in obs_nuisance]
#         self.cmd_nuisances = [instance(x) for x in cmd_nuisance]
#         # No - we should not call inverse() before transform_spec()
# 
#         obs_spec = self.robot.get_spec().get_observations()
#         for n in self.obs_nuisances:
#             obs_spec = n.transform_spec(obs_spec)
# 
#         cmd_spec = self.robot.get_spec().get_commands()
#         for n in self.cmd_nuisances:
#             cmd_spec = n.transform_spec(cmd_spec)
# 
#         # We don't really need to compute this...
#         try:
#             self.cmd_nuisances_inv = [x.inverse() for x in self.cmd_nuisances]
#     
#             # now initialize in reverse
#             cmd_spec_i = cmd_spec
#             for n in reversed(self.cmd_nuisances_inv):
#                 cmd_spec_i = n.transform_spec(cmd_spec_i)
#     
#             StreamSpec.check_same_spec(cmd_spec_i,
#                                    self.robot.get_spec().get_commands())
#             # TODO: why do we do this for commands, not for osbservations?
#         except Exception as e:
#             logger.warning('It seems that this chain of nuisances is not '
#                            'exact, but it could be OK to continue. '
#                            ' The chain is %s; the error is:\n%s' % 
#                            (cmd_nuisance, indent(str(e).strip(), '> ')))
# 
#         self.spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec)
#         self.obs_nuisances_id = obs_nuisance
#         self.cmd_nuisances_id = cmd_nuisance
#         
#     def __str__(self):
#         return self.desc
# 
#     def get_spec(self):
#         return self.spec
# 
#     def get_state(self):
#         return self.robot.get_state()
# 
#     def get_observations(self):
#         try:
#             obs = self.robot.get_observations()
#         except RobotObservations.NotReady:
#             raise 
# 
#         obs.observations = self._apply_nuisances_observations(obs.observations)
# 
#         try:
#             obs.commands = self._apply_nuisances_commands_inv(obs.commands)
#         except Exception as e:
#             warnings.warn('Something fishy here: %s' % e)
#             # FIXME: warn for this
#             pass
#         
#         return obs
# 
#     @contract(commands='array')
#     def set_commands(self, commands, commands_source):
#         for n in self.cmd_nuisances:
#             commands = n.transform_value(commands)
#         self.robot.set_commands(commands, commands_source)
# 
#     def new_episode(self):
#         return self.robot.new_episode()
# 
#     def get_inner_components(self):
#         return [self] + self.robot.get_inner_components()
#   
#     @deprecated
#     def get_original_robot(self):
#         """ Returns the robot that we are wrapping with this nuisance. """
#         if isinstance(self.robot, EquivRobot):
#             return self.robot.get_original_robot()
#         else:
#             return self.robot
#     
#     @contract(returns='tuple(tuple, str, tuple)')
#     def get_robot_modulus(self):
#         if isinstance(self.robot, EquivRobot):
#             obsr, his, cmdr = self.robot.get_robot_modulus()
#             
#             obs = obsr + tuple(self.obs_nuisances_id)
#             cmd = tuple(self.cmd_nuisances_id) + cmdr  # XXX: check order 
#             
#             return (obs, his, cmd)
#         else:
#             return (tuple(self.obs_nuisances_id),
#                     self.inner_robot_name,
#                     tuple(self.cmd_nuisances_id))
#     
#     @contract(returns='tuple(list(str),list(str))')
#     def get_nuisances(self):
#         """ Returns two lists of ID representing the nuisances 
#             applied to observations and commands. """
#         obs = list(self.obs_nuisances_id)
#         cmd = list(self.cmd_nuisances_id)
#         if isinstance(self.robot, EquivRobot):
#             obsr, cmdr = self.robot.get_nuisances()
#             obs = obsr + obs  # XXX: check order is correct
#             cmd = cmd + cmdr
#         return obs, cmd
#             
#     @contract(obs1='array')
#     def convert_observations_array(self, obs1):
#         """ Converts the observations from the inner to the output. """
#         # Look that 
#         dtype = get_observations_dtype(self.spec)
#         obs2 = np.zeros((), dtype)
#         for f in set(dtype.names) - set(['observations', 'commands', 'id_robot']):
#             obs2[f].flat = obs1[f].flat
#         
#         obs2['observations'] = self._apply_nuisances_observations(obs1['observations']) 
#         obs2['commands'] = self._apply_nuisances_commands_inv(obs1['commands'])
#         obs2['extra'] = obs1['extra']
#         obs2['id_robot'] = 'unset'
#         return obs2
#     
#     def debug_get_vel_from_commands(self, commands):
#         commands = self._apply_nuisances_commands(commands)
#         return self.robot.debug_get_vel_from_commands(commands)
#         
#     def _apply_nuisances_observations(self, values):
#         for n in self.obs_nuisances:
#             values = n.transform_value(values)
#         return values
#     
#     def _apply_nuisances_commands_inv(self, values):
#         # Note we use the inverted order
#         for n in reversed(self.cmd_nuisances_inv):
#             values = n.transform_value(values)
#         return values
# 
#     def _apply_nuisances_commands(self, commands):
#         for n in self.cmd_nuisances:
#             commands = n.transform_value(commands)
#         return commands
