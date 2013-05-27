from contracts import contract
from bootstrapping_olympics import (BootSpec, RobotInterface)

__all__ = ['DummyRobot', 'dummy_robot_from_spec']

class DummyRobot(RobotInterface):
    """ A dummy robot is one that has a spec, but none 
        of the meat is implemented. It is used to represent
        the robot that generated some log file.
    """
    def __init__(self, spec):
        """ :param spec: a BootSpec object """
        self.spec = spec
    
    @contract(returns=BootSpec)
    def get_spec(self):
        return self.spec 

    def new_episode(self):
        raise NotImplementedError()
    
    def set_commands(self, commands, commands_source):
        raise NotImplementedError()

    def get_observations(self):
        raise NotImplementedError()

    def get_state(self):
        raise NotImplementedError()

    def set_state(self, state):
        raise NotImplementedError()


def dummy_robot_from_spec(boot_spec_yaml):
    boot_spec = BootSpec.from_yaml(boot_spec_yaml)
    robot = DummyRobot(boot_spec)
    return robot
