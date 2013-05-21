from bootstrapping_olympics.interfaces.observations import ObsKeeper
from bootstrapping_olympics.interfaces.robot import RobotObservations
import time


def bd_sequence_from_robot_agent(id_robot, robot, id_agent, agent,
                                 sleep_wait, id_episode, id_environment,
                                 check_valid_values=False):
    """
        yields array of bd
    """
    boot_spec = robot.get_spec() 
    
    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=check_valid_values)

    for obs in iterate_robot_observations(robot, sleep_wait):
        bd = keeper.push(timestamp=obs.timestamp,
                                        observations=obs.observations,
                                        commands=obs.commands,
                                        commands_source=obs.commands_source,
                                        id_episode=id_episode,
                                        id_world=id_environment)

        yield bd

        agent.process_observations(bd)
        commands = agent.choose_commands()  # repeated
        robot.set_commands(commands, id_agent)



def bd_sequence_from_robot(id_robot, robot, sleep_wait, id_episode, id_environment,
                           check_valid_values=False):
    """
    
        :param robobs_seq: Sequence of RobotObservations
        
        :returns: iterator of bd array
    """
    
    boot_spec = robot.get_spec() 

    keeper = ObsKeeper(boot_spec=boot_spec, id_robot=id_robot,
                       check_valid_values=check_valid_values)

    for obs in iterate_robot_observations(robot, sleep_wait):
        bd = keeper.push(timestamp=obs.timestamp,
                                        observations=obs.observations,
                                        commands=obs.commands,
                                        commands_source=obs.commands_source,
                                        id_episode=id_episode,
                                        id_world=id_environment)
        yield bd

def iterate_robot_observations(robot, sleep=0.1):
    while True:
        try:
            yield robot.get_observations()
        except RobotObservations.NotReady:
            if sleep > 0:
                print('sleeping %s' % sleep)
                time.sleep(sleep)
                continue
            else:
                # just go to the next
                pass
                
        except RobotObservations.Finished:
            break
        
