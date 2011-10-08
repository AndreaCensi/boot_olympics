from bootstrapping_olympics import UnsupportedSpec
from bootstrapping_olympics.programs.manager.cmd_simulate import run_simulation

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
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return
    for _ in run_simulation(id_robot=id_robot,
                            robot=robot,
                            agent=agent,
                            max_observations=3, max_time=100):
        pass
