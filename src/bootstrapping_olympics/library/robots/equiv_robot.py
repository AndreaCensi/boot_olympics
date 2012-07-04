from . import contract, logger
from bootstrapping_olympics import (BootSpec, RobotInterface,
                                    BootOlympicsConfig, StreamSpec)
from bootstrapping_olympics.utils import indent

__all__ = ['EquivRobot']


class EquivRobot(RobotInterface):
    ''' This is used to create the perturbed version of a robot. '''

    @contract(robot='str',
              obs_nuisance='str|list(str)',
              cmd_nuisance='str|list(str)')
    def __init__(self, robot, obs_nuisance=[], cmd_nuisance=[]):
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

    def __str__(self):
        return self.desc

    def get_spec(self):
        return self.spec

    def get_state(self):
        return self.robot.get_state()

    def get_observations(self):
        obs = self.robot.get_observations()
        values = obs.observations
        for n in self.obs_nuisances:
            values = n.transform_value(values)
        obs.observations = values

        try:
            # Note we use the inverted order
            values = obs.commands
            for n in reversed(self.cmd_nuisances_inv):
                values = n.transform_value(values)
            obs.commands = values
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
        return self.robot

