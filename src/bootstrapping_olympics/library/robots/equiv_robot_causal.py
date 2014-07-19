# from blocks import SimpleBlackBox, Source, bb_get_block_poll_sleep, bb_pump
# from blocks.library import WithQueue
# from bootstrapping_olympics import (RobotInterface, RobotObservations, 
#     get_boot_config)
# from contracts import contract
# from contracts.utils import deprecated
# import warnings
# 
# 
# class EquivRobotCausal(RobotInterface):
#     """ This is the equivalent of NuisanceAgent for the robot """
#     
#     @staticmethod
#     @deprecated
#     def from_yaml(robot, nuisance):
#         return EquivRobotCausal(robot=robot, nuisance=nuisance)
#         
#     @contract(robot='str|code_spec|isinstance(RobotInterface)', 
#               nuisance='str|code_spec|isinstance(RepresentationNuisanceCausal)')
#     def __init__(self, robot, nuisance):
#         boot_config = get_boot_config()
#         _, self.robot = boot_config.robots.instance_smarter(robot)
#         _, self.nuisance = boot_config.nuisances_causal.instance_smarter(nuisance)
#         
#         self.spec1 = self.robot.get_spec()
#         self.spec2 = self.nuisance.transform_spec(self.spec1)
#          
#         self.pre = self.nuisance.get_pre()
#         self.robotw = RobotAsBlackBox(self.robot)
#         self.post = self.nuisance.get_post()
# 
#         self.log_add_child('pre', self.pre)
#         self.log_add_child('post', self.post)
#         self.log_add_child('robotw', self.robotw)
#         
#         self.last_obs = None
#         self.last_commands = None
#     
#     def get_spec(self):
#         return self.spec2
#     
#     def set_commands(self, commands, commands_source):
#         # need to get timestamp
#         if self.last_obs is None:
#             self.last_obs = self.robot.get_observations()
#         t = self.last_obs.timestamp
#         # self.info('EquivRobotCausal:set_commands()')
#         self.last_commands = (commands, commands_source)
#         self.pre.put((t, self.last_commands))
#         bb_pump(self.pre, self.robotw)
#           
#     def get_observations(self):
#         # self.info('EquivRobotCausal:get_observations()')
#         bb_pump(self.robotw, self.post)
#         try:
#             _, obs1 = self.post.get(block=False)
#             if self.last_commands is None:
#                 warnings.warn('Should get default value from spec otherwise fail')
#                 pass
#             else:
#                 obs1.commands, obs1.commands_source = self.last_commands
#             return obs1
#         except SimpleBlackBox.NotReady:
#             raise RobotObservations.NotReady()
#         
#     def new_episode(self):
#         return self.robot.new_episode()
# 
#     def get_inner_components(self):
#         return [self] + self.robot.get_inner_components()
#             
# 
# 
# 
# 
# 
# class RobotAsBlackBox(Source):
#     # TODO: remove in favor of RobotAsBlackBox2
#     @contract(robot=RobotInterface)
#     def __init__(self, robot, sleep=0.1):
#         self.log_add_child('robot', robot)
#         
#         self.sleep = sleep
#         self.robot = robot
#         self.last_obs = None
# 
#     def __repr__(self):
#         return 'RobotAsBlackBox(%r)' % self.robot
# 
#     def reset(self):
#         WithQueue.reset(self)
#         self.episode = self.robot.new_episode()
# 
# 
#     @contract(value='tuple(float,*)')
#     def put(self, value):
#         _, cmds = value
#         self.robot.set_commands(*cmds)
#        
#     def get(self, block=True, timeout=None):
#         if block:
#             return bb_get_block_poll_sleep(self, timeout=timeout,
#                                            sleep=self.sleep)
#         else:
#             return self._get_notblock()
#               
#     def _get_notblock(self):
#         try:
#             obs = self.robot.get_observations()
#         except RobotObservations.NotReady:
#             raise SimpleBlackBox.NotReady()
#      
#         if self.last_obs is not None:
#             if self.last_obs.timestamp == obs.timestamp:
#                 raise SimpleBlackBox.NotReady()
#                 
#         self.last_obs = obs
#             
#         cmd = obs.commands
#         cmd_source = obs.commands_source
#         return (obs.timestamp, ((cmd, cmd_source), obs))
# 
# 
# # TODO: remove this one
