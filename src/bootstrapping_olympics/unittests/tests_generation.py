from . import all_robots, get_robot, all_agents, get_agent, logger
import sys
from nose.tools import istest


def add_to_module(function, module_name):
    module = sys.modules[module_name]
    if not 'test' in module_name:
        logger.error('Warning: Nose will not find tests in %r.' % module_name)
    name = '%s_%s' % ('test', function.__name__)
    if name in module.__dict__:
        raise Exception('Already created test %r.' % name)
    module.__dict__[name] = function

    if not 'test' in module_name:
        raise Exception('While adding %r in %r: module does not have "test"'
                        ' in it, so nose will not find the test.' %
                        (name, module_name))


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
        wrap_with_desc(f, (id_agent, agent, id_robot, robot),
                       agent=agent, robot=robot)

    name = 'test_%s_%s_%s' % (f.__name__, id_agent, id_robot)
    test_caller.__name__ = name
    test_caller.agent = id_agent
    test_caller.robot = id_robot

    add_to_module(test_caller, f.__module__)


def for_all_pairs(f):
    for id_agent in all_agents():
        for id_robot in all_robots():
            add_pair_f(f, id_robot, id_agent)


def wrap_with_desc(function, arguments, agent=None, robot=None):
    ''' Calls function with arguments, and writes debug information
        if an exception is detected. '''

    try:
        function(*arguments)
    except:
        msg = ('Error detected when running test (%s); displaying debug info.'
               % function.__name__)
        if robot is not None:
            msg += '\nRobot: %s' % robot
            msg += '\n Obs spec: %s' % robot.get_spec().get_observations()
            msg += '\n Cmd spec: %s' % robot.get_spec().get_commands()
        if agent is not None:
            msg += '\nAgent: %s' % agent

        logger.error(msg)
        raise



