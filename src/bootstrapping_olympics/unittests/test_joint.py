from bootstrapping_olympics import UnsupportedSpec
from bootstrapping_olympics.ros_scripts.log_learn.cmd_simulate import run_simulation
from . import for_all_pairs

@for_all_pairs
def check_agent_init(id_agent, agent, id_robot, robot):
    spec = robot.get_spec()
    try:
        agent.init(spec)
    except UnsupportedSpec:
        pass

@for_all_pairs
def check_small_simulation(id_agent, agent, id_robot, robot):
    for x in run_simulation(robot, agent, 3, 1000):
        print(x['counter'])
