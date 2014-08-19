# from bootstrapping_olympics import (RobotInterface, RobotObservations, ObsKeeper,
#     logger)
# from contracts import contract
# 
# # FIXME: should be the same as run_simulation()
# __all__ = ['run_simulation_servo']
# 
# @contract(id_robot='str', id_agent='str',
#           robot=RobotInterface, max_observations='>=1',
#           max_time='>0')
# def run_simulation_servo(id_robot, robot, id_agent, agent,
#                          max_observations, max_time,
#                          id_episode, id_environment,
#                          check_valid_values=True):
#     ''' Runs an episode of the simulation. The agent should already been
#         init()ed. '''
# 
#     keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot,
#                        check_valid_values=check_valid_values)
# 
#     obs_spec = robot.get_spec().get_observations()
#     cmd_spec = robot.get_spec().get_commands()
# 
#     counter = 0
# 
#     while counter < max_observations:
# 
#         obs = robot.get_observations()
#         assert isinstance(obs, RobotObservations)
# 
#         if check_valid_values:
#             obs_spec.check_valid_value(obs.observations)
# 
#         observations = keeper.push(timestamp=obs.timestamp,
#                                    observations=obs.observations,
#                                    commands=obs.commands,
#                                    commands_source=obs.commands_source,
#                                    id_episode=id_episode,
#                                    id_world=id_environment)
#         episode_end = obs.episode_end
# 
#         yield obs, observations
# 
#         if observations['time_from_episode_start'] > max_time:
#             logger.debug('Episode ended at %s for time limit %s > %s ' % 
#                          (counter, observations['time_from_episode_start'],
#                           max_time))
#             break
# 
#         if episode_end:  # Fishy
#             logger.debug('Episode ended at %s due to obs.episode_end.'
#                          % counter)
#             break
# 
#         agent.process_observations(observations)
#         commands = agent.choose_commands()  # repeated
# 
#         if check_valid_values:
#             cmd_spec.check_valid_value(commands)
# 
#         robot.set_commands(commands, id_agent)
# 
#         counter += 1
