from . import all_robots, get_robot, all_agents, get_agent, logger
import sys


module = sys.modules[__name__]

def for_all_robots(f):
    for id_robot in all_robots():
        def test_caller():
            robot = get_robot(id_robot)
            wrap_with_desc(f, (id_robot, robot), agent=None, robot=robot)

        name = '%s-%s' % (f.__name__, id_robot)
        test_caller.__name__ = name
        module.__dict__[name] = test_caller
    return None

def for_all_agents(f):
    for id_agent in all_agents():
        def test_caller():
            agent = get_agent(id_agent)
            wrap_with_desc(f, (id_agent, agent), agent=agent, robot=None)
            
        name = '%s-%s' % (f.__name__, id_agent)
        test_caller.__name__ = name
        module.__dict__[name] = test_caller
    return None


def for_all_pairs(f):
    for id_agent in all_agents():
        for id_robot in all_robots():
            def test_caller():
                agent = get_agent(id_agent)
                robot = get_robot(id_robot)
                wrap_with_desc(f, (id_agent, agent, id_robot, robot),
                               agent=agent, robot=robot)
                
            name = '%s-%s' % (f.__name__, id_agent)
            test_caller.__name__ = name
            module.__dict__[name] = test_caller
    return None

def wrap_with_desc(function, arguments, agent=None, robot=None):
    ''' Calls function with arguments, and writes debug information
        if an exception is detected. '''
    
    try:
        function(*arguments)
    except:
        msg = ('Error detected when running test (%s); displaying debug info.' 
               % function.__name__)
        if robot is not None:
            msg += '\n Robot: %s' % robot
            msg += '\n Obs spec: %s' % robot.get_spec().get_observations()
            msg += '\n Cmd spec: %s' % robot.get_spec().get_commands()
            logger.error(msg)
        # TODO: agent
        raise 
    
    
    
    
