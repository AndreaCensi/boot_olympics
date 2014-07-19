# import unittest
# 
# from numpy.testing.utils import assert_almost_equal
# 
# from bootstrapping_olympics import RobotObservations
# from bootstrapping_olympics.library.agents import RandomAgent
# from bootstrapping_olympics.library.nuisances_causal import PureCommandsNuisance
# from bootstrapping_olympics.library.robots import EquivRobotCausal, RandomRobot
# from streamels import BootSpec, StreamSpec, make_streamels_rgb_float, make_streamels_1D_float
# 
# # 
# # class PureCommandsTest(unittest.TestCase):
# #     
# #     def test_pure_commands(self):
# #         obs = StreamSpec(id_stream=None, streamels=make_streamels_rgb_float((10, 10)))
# #         cmd = StreamSpec(id_stream=None, streamels=make_streamels_1D_float(2, 0.0, 1.0))
# #         
# #         delta = 1.0
# #         n = 10  # in this case we have to match everything
# #         nuisance = PureCommandsNuisance(delta=delta, n=n)
# #         r0 = RandomRobot(BootSpec(obs, cmd), t0=0.0)
# #         robot = EquivRobotCausal(r0, nuisance)
# #         
# #         agent = RandomAgent()
# #         agent.init(robot.get_spec())
# #         
# #         # first observations should be available
# #         rest = robot.get_spec().get_commands().get_default_value()
# #         obs = robot.get_observations()
# #         
# #         self.assertEqual(obs.timestamp, 0)
# #         # now it is not available
# #         self.assertRaises(RobotObservations.NotReady, robot.get_observations)
# # 
# #         for i in range(4):
# #             robot.set_commands(rest, 'rest')
# #             obs = robot.get_observations()
# #             assert_almost_equal(obs.timestamp, delta * (i + 1))
# #             self.assertRaises(RobotObservations.NotReady, robot.get_observations)
# #             
