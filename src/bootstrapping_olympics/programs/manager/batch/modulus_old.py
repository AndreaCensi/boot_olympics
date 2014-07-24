# 
#     def add_tasks_explore_modulus(self, robots, episodes_per_tranche=50, **explore_args):
#         """ 
#             Adds exploration tasks, making sure to use the same 
#             data for robots up to nuisances. 
#         """
#         # ID robot -> (obsn, original, cmdn)
#         robot2canonical = {}
#         
#         # (original, cmd) -> list(str)
#         from collections import defaultdict
#         core2robots = defaultdict(list)
#         
#         for id_robot in robots:
#             canform = self.get_robot_modulus(id_robot)
#             robot2canonical[id_robot] = canform
#             _, original, cmdn = canform
#             core2robots[(original, cmdn)].append(id_robot)
#             
#             logger.info('Canonical form of %r: %s' % 
#                         (id_robot, robot2canonical[id_robot]))
#             
#         config = self.data_central.get_bo_config()
#         
#         for original, cmd in core2robots:
#             derived = core2robots[(original, cmd)]
#             logger.info('Core robot (%s,%s) corresponds to %s' % 
#                         (original, cmd, derived))
#             
#             # XXX: not sure of order
#             new_robot_name = "".join(['U%s' % x for x in cmd]) + original
#             if not new_robot_name in config.robots:
#                 msg = 'Bug found, or bad naming practices.'
#                 msg += 'I want to instantiate a %r but not found.' % new_robot_name
#                 raise Exception(msg)
#             
#             explore_args['episodes_per_tranche'] = episodes_per_tranche
#             id_episodes = self.add_tasks_explore(new_robot_name, **explore_args)
#             logger.info('Defined %d episodes for %r.' % (len(id_episodes), new_robot_name))
#             
#             # Now convert one episode into the other
#             for id_derived in derived:
#                 logger.info('Considering derived %s ' % id_derived)
#                 derived_obs, x, derived_cmd = robot2canonical[id_derived]
#                 assert x == original
#                 assert derived_cmd == cmd
#                 if not derived_obs:
#                     logger.info(' ... skipping because pure.')
#                     # this is a pure robot
#                     continue
#                 
#                 episodes_tranches = self.get_tranches(id_episodes,
#                                                       episodes_per_tranche)
#                 
#                 for i, tranche in enumerate(episodes_tranches):
#                     job_id = 'derive-%s-%d' % (id_derived, i)
#                     
#                     extra_dep = []
#                     for id_episode in tranche:
#                         extra_dep.append(self.dep_episode_done(new_robot_name,
#                                                                id_episode))
#                         
#                     job = self.compmake_job(nuislog_episodes,
#                                             data_central=self.data_central,
#                                             id_robot_original=new_robot_name,
#                                             id_episodes=tranche,
#                                             id_equiv=id_derived,
#                                             obs_nuisances=derived_obs,
#                                             cmd_nuisances=[],  # <- correct
#                                             with_extras=True,
#                                             job_id=job_id,
#                                             extra_dep=extra_dep)
#                     
#                     for id_episode in tranche:
#                         self.set_dep_episode_done(id_robot=id_derived,
#                                                   id_episode=id_episode, job=job)
#      
#     
#     @contract(returns='tuple(tuple, str, tuple)')
#     def get_robot_modulus(self, id_robot):
#         """ Returns the canonical description of a robot,
#             as a list of nuisances on observations,
#             robot name,
#             nuisances on commands. 
#         
#             obsn, original, cmdn = self.get_robot_modulus() 
#         """
#         
#         robot = self._get_robot_instance(id_robot)
#         if isinstance(robot, EquivRobot):
#             return robot.get_robot_modulus()
#         else:
#             return (tuple([]), id_robot, tuple([]))