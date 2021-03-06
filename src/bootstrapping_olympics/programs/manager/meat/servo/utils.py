from bootstrapping_olympics.library.robots.equiv_robot import EquivRobot
from contracts import describe_type

__all__ = ['get_vsim_from_robot']

def get_vsim_from_robot(robot):
    """ Checks that the robot is a VehicleSimulation
        or an EquivRobot. If not, it raises an exception.
        Returns an instance to the VehicleSimulation. """
    if isinstance(robot, EquivRobot):
        robot = robot.get_original_robot()
        return get_vsim_from_robot(robot)

    from vehicles_boot import BOVehicleSimulation
    if isinstance(robot, BOVehicleSimulation):
        return robot

    msg = ('I require that the robot is a VehiclesSimulation, '
           'but obtained %s' % describe_type(robot))
    raise ValueError(msg)
