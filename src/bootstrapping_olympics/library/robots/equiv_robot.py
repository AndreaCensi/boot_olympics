from . import contract, logger
from bootstrapping_olympics import (BootSpec, RobotInterface,
                                    BootOlympicsConfig, StreamSpec)
from bootstrapping_olympics.utils import indent
from bootstrapping_olympics.interfaces.observations import get_observations_dtype
import numpy as np

__all__ = ['EquivRobot']


class EquivRobot(RobotInterface):
    ''' This is used to create the perturbed version of a robot. '''

    @contract(robot='str',
              obs_nuisance='str|list(str)',
              cmd_nuisance='str|list(str)')
    def __init__(self, robot, obs_nuisance=[], cmd_nuisance=[]):
        self.inner_robot_name = robot 
        # todo: use get_current_bo_config()
        self.robot = BootOlympicsConfig.specs['robots'].instance(robot)

        self.desc = ('EquivRobot(%s,obs:%s,cmd:%s)'
                    % (robot, obs_nuisance, cmd_nuisance))

        # convert to (possibly empty) list of strings
        if isinstance(obs_nuisance, str):
            obs_nuisance = [obs_nuisance]
        if isinstance(cmd_nuisance, str):
            cmd_nuisance = [cmd_nuisance]

        instance = BootOlympicsConfig.specs['nuisances'].instance
        self.obs_nuisances = [instance(x) for x in obs_nuisance]
        self.cmd_nuisances = [instance(x) for x in cmd_nuisance]
        # No - we should not call inverse() before transform_spec()

        obs_spec = self.robot.get_spec().get_observations()
        for n in self.obs_nuisances:
            obs_spec = n.transform_spec(obs_spec)

        cmd_spec = self.robot.get_spec().get_commands()
        for n in self.cmd_nuisances:
            cmd_spec = n.transform_spec(cmd_spec)

        # We don't really need to compute this...
        try:
            self.cmd_nuisances_inv = [x.inverse() for x in self.cmd_nuisances]
    
            # now initialize in reverse
            cmd_spec_i = cmd_spec
            for n in reversed(self.cmd_nuisances_inv):
                cmd_spec_i = n.transform_spec(cmd_spec_i)
    
            StreamSpec.check_same_spec(cmd_spec_i,
                                   self.robot.get_spec().get_commands())
            # TODO: why do we do this for commands, not for osbservations?
        except Exception as e:
            logger.warning('It seems that this chain of nuisances is not '
                           'exact, but it could be OK to continue. '
                           ' The chain is %s; the error is:\n%s' % 
                           (cmd_nuisance, indent(str(e).strip(), '> ')))

        self.spec = BootSpec(obs_spec=obs_spec, cmd_spec=cmd_spec)
        self.obs_nuisances_id = obs_nuisance
        self.cmd_nuisances_id = cmd_nuisance
        
    def __str__(self):
        return self.desc

    def get_spec(self):
        return self.spec

    def get_state(self):
        return self.robot.get_state()

    def get_observations(self):
        obs = self.robot.get_observations()

        obs.observations = self._apply_nuisances_observations(obs.observations)

        try:
            obs.commands = self._apply_nuisances_commands_inv(obs.commands)
        except:
            # FIXME: warn for this
            pass
        
        return obs

    @contract(commands='array')
    def set_commands(self, commands, commands_source):
        for n in self.cmd_nuisances:
            commands = n.transform_value(commands)
        self.robot.set_commands(commands, commands_source)

    def new_episode(self):
        return self.robot.new_episode()

    def get_original_robot(self):
        """ Returns the robot that we are wrapping with this nuisance. """
        if isinstance(self.robot, EquivRobot):
            return self.robot.get_original_robot()
        else:
            return self.robot
    
    @contract(returns='tuple(tuple, str, tuple)')
    def get_robot_modulus(self):
        if isinstance(self.robot, EquivRobot):
            obsr, his, cmdr = self.robot.get_robot_modulus()
            
            obs = obsr + tuple(self.obs_nuisances_id)
            cmd = tuple(self.cmd_nuisances_id) + cmdr # XXX: check order 
            
            return (obs, his, cmd)
        else:
            return (tuple(self.obs_nuisances_id),
                    self.inner_robot_name,
                    tuple(self.cmd_nuisances_id))
    
    @contract(returns='tuple(list(str),list(str))')
    def get_nuisances(self):
        """ Returns two lists of ID representing the nuisances 
            applied to observations and commands. """
        obs = list(self.obs_nuisances_id)
        cmd = list(self.cmd_nuisances_id)
        if isinstance(self.robot, EquivRobot):
            obsr, cmdr = self.robot.get_nuisances()
            obs = obsr + obs # XXX: check order is correct
            cmd = cmd + cmdr
        return obs, cmd
            
            
    @contract(obs1='array')
    def convert_observations_array(self, obs1):
        """ Converts the observations from the inner to the output. """
        # Look that 
        dtype = get_observations_dtype(self.spec)
        obs2 = np.zeros((), dtype)
        for f in set(dtype.names) - set(['observations', 'commands', 'id_robot']):
            obs2[f].flat = obs1[f].flat
        
        obs2['observations'] = self._apply_nuisances_observations(obs1['observations']) 
        obs2['commands'] = self._apply_nuisances_commands_inv(obs1['commands'])
        obs2['extra'] = obs1['extra']
        obs2['id_robot'] = 'unset'
        return obs2
    
    def _apply_nuisances_observations(self, values):
        for n in self.obs_nuisances:
            values = n.transform_value(values)
        return values
    
    def _apply_nuisances_commands_inv(self, values):
        # Note we use the inverted order
        for n in reversed(self.cmd_nuisances_inv):
            values = n.transform_value(values)
        return values

