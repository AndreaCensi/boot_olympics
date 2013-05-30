from bootstrapping_olympics import (get_conftools_agents, get_conftools_robots,
    get_conftools_nuisances, get_conftools_nuisances_causal)
from comptests import comptests_for_all_pairs, comptests_for_all


library_agents = get_conftools_agents()
library_robots = get_conftools_robots()
library_nuisances = get_conftools_nuisances()
library_nuisances_causal = get_conftools_nuisances_causal()

for_all_robots = comptests_for_all(library_robots)
for_all_agents = comptests_for_all(library_agents)
for_all_nuisances = comptests_for_all(library_nuisances)
for_all_nuisances_causal = comptests_for_all(library_nuisances_causal)

for_all_pairs = comptests_for_all_pairs(library_agents, library_robots)
for_all_robot_nuisance_pairs = comptests_for_all_pairs(library_robots, library_nuisances)



# 
# # XXX: this is not used yet
# def wrap_with_desc(function, arguments,
#                    agent=None, robot=None, nuisance=None):
#     ''' Calls function with arguments, and writes debug information
#         if an exception is detected. '''
# 
#     try:
#         function(*arguments)
#     except:
#         msg = ('Error detected when running test (%s); '
#                'displaying debug info.'
#                % function.__name__)
#         if robot is not None:
#             msg += '\nRobot: %s' % robot
#             msg += '\n Obs spec: %s' % robot.get_spec().get_observations()
#             msg += '\n Cmd spec: %s' % robot.get_spec().get_commands()
#         if agent is not None:
#             msg += '\nAgent: %s' % agent
#         if nuisance is not None:
#             msg += '\nAgent: %s' % nuisance
# 
#         logger.error(msg)
#         raise



