from bootstrapping_olympics.examples.robots.equiv_robot import EquivRobot
from contracts import describe_type


def get_vsim_from_robot(robot):
    """ Checks that the robot is a VehicleSimulation
        or an EquivRobot. If not, it raises an exception.
        Returns an instance to the VehicleSimulation. """
    if isinstance(robot, EquivRobot):
        robot = robot.get_original_robot()

    from vehicles import VehicleSimulation
    if isinstance(robot, VehicleSimulation):
        return robot

    msg = ('I require that the robot is a VehiclesSimulation, '
           'but obtained %s' % describe_type(robot))
    raise ValueError(msg)
