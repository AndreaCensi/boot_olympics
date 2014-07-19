from blocks.library.simple.identity import Identity
from bootstrapping_olympics.library.agents.nuisance_agent_actions import (
    wrap_robot_exploration)
from bootstrapping_olympics.unittests.tests_generation import (
    for_all_nuisances_causal, for_all_robot_rnc_pairs)
from comptests.results import Skipped
from streamels.exceptions import UnsupportedSpec
from bootstrapping_olympics.interfaces.robot import ExplorableRobot
from blocks.library.timed.checks import check_timed_named


@for_all_nuisances_causal
def check_rnc(id_rnc, rnc):  # @UnusedVariable
    
    try:
        rnc.get_H()
    except ValueError:
        pass
    else:
        msg = 'get_H() should throw exception if transform_spec() not called.'
        raise ValueError(msg)
    
    
    try:
        rnc.get_G()
    except ValueError:
        pass
    else:
        msg = 'get_G() should throw exception if transform_spec() not called.'
        raise ValueError(msg)
    
    try:
        rnc.get_H_conj()
    except ValueError:
        pass
    else:
        msg = 'get_H_conj() should throw exception if transform_spec() not called.'
        raise ValueError(msg)
    
    
    try:
        rnc.get_H()
    except ValueError:
        pass
    else:
        msg = 'get_H() should throw exception if transform_spec() not called.'
        raise ValueError(msg)
        
      
    
@for_all_robot_rnc_pairs
def check_rnc_init(id_robot, robot, id_rnc, rnc):  # @UnusedVariable
    boot_spec = robot.get_spec()
    
    try:
        rnc.transform_spec(boot_spec)
    except UnsupportedSpec:
        msg = 'UnsupportedSpec'
        print(msg)
        return Skipped(msg)
    
    H = rnc.get_H()
    H_conj = rnc.get_H_conj()
    G = rnc.get_G()
    G_conj = rnc.get_G_conj()
    
    
    
@for_all_robot_rnc_pairs
def check_rnc_wrap_active(id_robot, robot, id_rnc, rnc):  # @UnusedVariable
    if not isinstance(robot, ExplorableRobot):
        msg = 'Robot not explorable'
        print(msg)
        return Skipped(msg)
    
    robot_sys = robot.get_active_stream() 
    
    boot_spec = robot.get_spec()
    try:
        rnc.transform_spec(boot_spec)
    except UnsupportedSpec:
        msg = 'UnsupportedSpec'
        print(msg)
        return Skipped(msg)

    robot_sys2 = wrap_robot_exploration(robot_sys, rnc)
    
    
    rest = boot_spec.get_commands().get_default_value()
    robot_sys2.put((0, ('commands', rest)))
    
    x = robot_sys.get(block=True)
    check_timed_named(x)
    
    
    
