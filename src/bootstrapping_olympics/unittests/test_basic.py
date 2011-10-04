from .instantiation import all_agents, all_robots
from bootstrapping_olympics import (BootSpec, AgentInterface, BootOlympicsConfig,
    RobotInterface)

def check_agent_type(id_agent):
    agent = BootOlympicsConfig.agents.instance(id_agent) #@UndefinedVariable
    assert isinstance(agent, AgentInterface)

def agents_basic_type_test():
    for id_agent in all_agents():
        yield check_agent_type, id_agent
    
def check_robot_type(id_robot):
    robot = BootOlympicsConfig.robots.instance(id_robot) #@UndefinedVariable
    assert isinstance(robot, RobotInterface)

def check_robot_spec(id_robot):
    robot = BootOlympicsConfig.robots.instance(id_robot) #@UndefinedVariable
    cascade_spec = robot.get_spec()
    assert isinstance(cascade_spec, BootSpec)
    
def robots_basic_type_test():
    for id_robot in all_robots():
        yield check_robot_type, id_robot
        yield check_robot_spec, id_robot
