# from .robot import BasicRobot
# from blocks import NotReady
# from blocks.library import WithQueue
# from blocks.library.timed.checks import check_timed_named
# from bootstrapping_olympics import RobotObservations
# from contracts import contract
# 
# __all__ = [
#     'RobotAsBlackBox3',
# ]
# 
# class RobotAsBlackBox3(WithQueue):
#     """ This one does not look at handling NotReady correctly. 
#     
#         One observations is put in the queue when reset() is claled.
#         Then every time we got new_commands using put, we put new observations.
#     """
# 
#     # TODO: remove in favor of RobotAsBlackBox
#     @contract(robot=BasicRobot)
#     def __init__(self, robot):
#         self.log_add_child('robot', robot)
# 
#         self.robot = robot
#         self.last_obs = None
#         self.id_robot = 'unnamed_robot'
#         self.id_episode = None
# 
#     def reset(self):
#         WithQueue.reset(self)
# 
#         self.episode = self.robot.new_episode()
#         self.ended = False
#         #self._enqueue()
# 
#     def _enqueue(self):
#         try:
#             obs = self.robot.get_observations()
#         except NotReady as e:
#             msg = 'Sorry, RobotAsBlackBox3 does not handle NotReady.'
#             msg += '\n Not ready: %s' % e
#             raise NotReady(msg)
# 
#         if self.last_obs is not None:
#             if self.last_obs.timestamp == obs.timestamp:
#                 raise NotReady('got same obs')
# 
#         self.last_obs = obs 
#         self.append((obs.timestamp, ('observations', obs.observations)))
# 
#         self.ended = obs.episode_end
#     
#         if self.ended:
#             self.end_input()
#             
#     def __repr__(self):
#         return 'RobotAsBlackBox(%r)' % self.robot
# 
#     @contract(value='tuple(float,*)')
#     def put_noblock(self, value):
#         if self.ended:
#             msg = 'Calling put() after episode ended.'
#             raise ValueError(msg)
# 
#         check_timed_named(value, self)
#         (_, (sname, cmds)) = value
#         if not sname == 'commands':
#             msg = 'Expected signal "commands", not %r.' % sname
#             raise ValueError(sname)
#         
#         self.robot.set_commands(cmds, 'no-source-specified')
#         self._enqueue()
#         
#     
#         

