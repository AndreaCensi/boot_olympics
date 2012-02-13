from . import (all_nuisances, get_nuisance, all_robots, get_robot,
               all_agents, get_agent, logger)
from bootstrapping_olympics import UnsupportedSpec
from nose.tools import istest
import sys


def add_to_module(function, module_name):
    module = sys.modules[module_name]
    name = function.__name__

    if not 'test' in name:
        raise Exception('No "test" in function name %r' % name)

    if not 'test' in module_name:
        raise Exception('While adding %r in %r: module does not have "test"'
                        ' in it, so nose will not find the test.' %
                        (name, module_name))

    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)

    module.__dict__[name] = function


def add_robot_f(f, id_robot):
    @istest
    def test_caller():
        robot = get_robot(id_robot)
        wrap_with_desc(f, (id_robot, robot), agent=None, robot=robot)

    name = 'test_%s_%s' % (f.__name__, id_robot)
    test_caller.__name__ = name
    test_caller.robot = id_robot

    add_to_module(test_caller, f.__module__)


def for_all_robots(f):
    for id_robot in all_robots():
        add_robot_f(f, id_robot)


def add_agent_f(f, id_agent):
    @istest
    def test_caller():
        agent = get_agent(id_agent)
        wrap_with_desc(f, (id_agent, agent), agent=agent, robot=None)

    name = 'test_%s_%s' % (f.__name__, id_agent)
    test_caller.__name__ = name
    test_caller.agent = id_agent

    add_to_module(test_caller, f.__module__)


def for_all_agents(f):
    for id_agent in all_agents():
        add_agent_f(f, id_agent)


def add_pair_f(f, id_robot, id_agent):
    @istest
    def test_caller():
        agent = get_agent(id_agent)
        robot = get_robot(id_robot)

        try:
            agent1 = get_agent(id_agent)
            agent1.init(robot.get_spec())
        except UnsupportedSpec:
            print('Unsupported combination (%s,%s)' % (id_robot, id_agent))
            return

        wrap_with_desc(f, (id_agent, agent, id_robot, robot),
                       agent=agent, robot=robot)

    name = 'test_%s_%s_%s' % (f.__name__, id_agent, id_robot)
    test_caller.__name__ = name
    test_caller.agent = id_agent
    test_caller.robot = id_robot

    add_to_module(test_caller, f.__module__)


def add_robot_nuisance_pair_f(f, id_robot, id_nuisance):
    @istest
    def test_caller():
        nuisance = get_nuisance(id_nuisance)
        robot = get_robot(id_robot)
#
#        try:
#            agent1.init(robot.get_spec())
#        except UnsupportedSpec:
#            print('Unsupported combination (%s,%s)' % (id_robot, id_agent))
#            return

        wrap_with_desc(f, (id_robot, robot, id_nuisance, nuisance),
                       nuisance=nuisance, robot=robot)

    name = 'test_%s_%s_%s' % (f.__name__, id_nuisance, id_robot)
    test_caller.__name__ = name
    test_caller.agent = id_nuisance
    test_caller.robot = id_robot

    add_to_module(test_caller, f.__module__)


def for_all_pairs(f):
    for id_agent in all_agents():
        for id_robot in all_robots():
            add_pair_f(f, id_robot, id_agent)


def for_all_robot_nuisance_pairs(f):
    for id_nuisance in all_nuisances():
        for id_robot in all_robots():
            add_robot_nuisance_pair_f(f, id_robot, id_nuisance)


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



