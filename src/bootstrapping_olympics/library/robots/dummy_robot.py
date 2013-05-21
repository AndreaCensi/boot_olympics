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
        raise NotImplemented()
    
    def set_commands(self, commands, commands_source):
        raise NotImplemented()

    def get_observations(self):
        raise NotImplemented()

    def get_state(self):
        raise NotImplemented()

    def set_state(self, state):
        raise NotImplemented()


def dummy_robot_from_spec(boot_spec_yaml):
    boot_spec = BootSpec.from_yaml(boot_spec_yaml)
    robot = DummyRobot(boot_spec)
    return robot
