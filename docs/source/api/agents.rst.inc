[?1034hinherit contracts for WithQueue:get() from SimpleBlackBox
inherit contracts for RandomRobot:set_commands() from RobotInterface
inherit contracts for RandomRobot:get_spec() from PassiveRobotInterface
inherit contracts for RandomRobot:new_episode() from RobotInterface
inherit contracts for RandomRobot:get_observations() from PassiveRobotInterface
inherit contracts for TestRobot:set_commands() from RobotInterface
inherit contracts for TestRobot:get_spec() from PassiveRobotInterface
inherit contracts for TestRobot:new_episode() from RobotInterface
inherit contracts for TestRobot:get_observations() from PassiveRobotInterface
inherit contracts for DummyRobot:set_state() from RobotInterface
inherit contracts for DummyRobot:set_commands() from RobotInterface
inherit contracts for DummyRobot:get_spec() from PassiveRobotInterface
inherit contracts for DummyRobot:new_episode() from RobotInterface
inherit contracts for DummyRobot:get_state() from RobotInterface
inherit contracts for DummyRobot:get_observations() from PassiveRobotInterface
inherit contracts for EquivRobot:set_commands() from RobotInterface
inherit contracts for EquivRobot:get_spec() from PassiveRobotInterface
inherit contracts for EquivRobot:new_episode() from RobotInterface
inherit contracts for EquivRobot:get_state() from RobotInterface
inherit contracts for EquivRobot:get_observations() from PassiveRobotInterface
inherit contracts for EquivRobot:get_inner_components() from PassiveRobotInterface
inherit contracts for EquivRobotCausal:set_commands() from RobotInterface
inherit contracts for EquivRobotCausal:get_spec() from PassiveRobotInterface
inherit contracts for EquivRobotCausal:new_episode() from RobotInterface
inherit contracts for EquivRobotCausal:get_observations() from PassiveRobotInterface
inherit contracts for EquivRobotCausal:get_inner_components() from PassiveRobotInterface
inherit contracts for PureCommandsNuisance:transform_spec() from RepresentationNuisanceCausal
inherit contracts for PureCommandsNuisance:get_pre() from RepresentationNuisanceCausal
inherit contracts for PureCommandsNuisance:get_post() from RepresentationNuisanceCausal

.. py:module:: bootstrapping_olympics.library.agents

.. autosummary::  
   :toctree: api
   
   RandomAgent
   RandomAgentPredictor
   TestAgent



