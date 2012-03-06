from . import (all_nuisances, get_nuisance, all_robots, get_robot, all_agents,
    get_agent, logger, fancy_test_decorator)
import itertools


for_all_robots = fancy_test_decorator(lister=all_robots,
            arguments=lambda id_robot: (id_robot, get_robot(id_robot)),
            attributes=lambda id_robot: dict(robot=id_robot))

for_all_agents = fancy_test_decorator(lister=all_agents,
            arguments=lambda id_agent: (id_agent, get_agent(id_agent)),
            attributes=lambda id_agent: dict(agent=id_agent))

# from bootstrapping_olympics import UnsupportedSpec
# TODO: make sure this is fine
for_all_pairs = fancy_test_decorator(
            lister=lambda: itertools.product(all_agents(), all_robots()),
            arguments=lambda (id_agent, id_robot):
                 (id_agent, get_agent(id_agent),
                  id_robot, get_robot(id_robot)),
            attributes=lambda (id_agent, id_robot):
                dict(robot=id_robot, agent=id_agent),
                naming=lambda (a, b): '%s-%s' % (a, b))

for_all_robot_nuisance_pairs = fancy_test_decorator(
            lister=lambda: itertools.product(all_nuisances(), all_robots()),
            arguments=lambda (id_nuisance, id_robot):
                 (id_robot, get_robot(id_robot),
                  id_nuisance, get_nuisance(id_nuisance)),
            attributes=lambda (id_nuisance, id_robot):
                dict(robot=id_robot, nuisance=id_nuisance),
                naming=lambda (a, b): '%s-%s' % (a, b))


# XXX: this is not used yet
def wrap_with_desc(function, arguments,
                   agent=None, robot=None, nuisance=None):
    ''' Calls function with arguments, and writes debug information
        if an exception is detected. '''

    try:
        function(*arguments)
    except:
        msg = ('Error detected when running test (%s); '
               'displaying debug info.'
               % function.__name__)
        if robot is not None:
            msg += '\nRobot: %s' % robot
            msg += '\n Obs spec: %s' % robot.get_spec().get_observations()
            msg += '\n Cmd spec: %s' % robot.get_spec().get_commands()
        if agent is not None:
            msg += '\nAgent: %s' % agent
        if nuisance is not None:
            msg += '\nAgent: %s' % nuisance

        logger.error(msg)
        raise



