from .tests_generation import for_all_robots
from blocks import NeedInput, SimpleBlackBox
from blocks.library.timed.checks import check_timed_named
from bootstrapping_olympics import (BasicRobot, BootSpec, EpisodeDesc, 
    ExplorableRobot, StreamSpec)
from comptests.results import Skipped
from contracts import describe_type
from contracts.utils import check_isinstance
from numpy.ma.testutils import assert_not_equal
from types import NoneType
import yaml


@for_all_robots
def check_robot_type(id_robot, robot): #@UnusedVariable
    assert isinstance(robot, BasicRobot)


@for_all_robots
def check_robot_spec(id_robot, robot): #@UnusedVariable
    cascade_spec = robot.get_spec()
    assert isinstance(cascade_spec, BootSpec), describe_type(cascade_spec)

    cmd = cascade_spec.get_commands()
    assert isinstance(cmd, StreamSpec)
    cmd.check_valid_value(cmd.get_default_value())
    cmd.check_valid_value(cmd.get_random_value())

    obs = cascade_spec.get_observations()
    assert isinstance(obs, StreamSpec), describe_type(obs)
    obs.check_valid_value(obs.get_default_value())
    obs.check_valid_value(obs.get_random_value())


@for_all_robots
def check_robot_new_episode(id_robot, robot): #@UnusedVariable
    ed = robot.new_episode()
    assert isinstance(ed, EpisodeDesc), describe_type(ed)


@for_all_robots
def check_robot_new_episode_id(id_robot, robot): #@UnusedVariable
    ''' Episodes ID must be unique. '''
    ep1 = robot.new_episode()
    ep2 = robot.new_episode()
    assert_not_equal(ep1.id_episode, ep2.id_episode)

@for_all_robots
def check_robot_observations_compliance(id_robot, robot): #@UnusedVariable
    if not isinstance(robot, ExplorableRobot):
        msg = 'Robot not ExplorableRobot: %s' % describe_type(robot)
        print(msg)
        return Skipped(msg)
    
    try:
        stream = robot.get_active_stream()
    except NotImplementedError as e:
        msg = 'Not actually exploring: %s' %e
        print(msg)
        return Skipped(msg)
    
    check_isinstance(stream, SimpleBlackBox)
    stream.reset()
    stream.set_name_for_log('stream')
    
    # TODO: check that is a simulation, not an actual robot
    
    try:
        res_ = stream.get(block=True)
    except NeedInput:
        pass
    else:
        msg = 'Expected it would raise NeedInput without any commands.'
        msg += ' Actually got %r. ' % str(res_)
        raise Exception(msg)
    
    rest = robot.get_spec().get_commands().get_default_value()
    print(stream)
    stream.put((3.0, ('commands', rest)))
    
    res = stream.get(block=True)
    
    check_timed_named(res)
    t, (signal, obs) = res  # @UnusedVariable
    
    if not signal == 'observations':
        msg = 'Expected "observations", got %r.' % signal
        raise Exception(msg)
    
    obs_spec = robot.get_spec().get_observations()
    obs_spec.check_valid_value(obs)
    
    
@for_all_robots
def check_robot_state(id_robot, robot): #@UnusedVariable
    robot.new_episode() # always start an episode before getting observations
    state = robot.get_state()
    assert isinstance(state, (NoneType, dict))


@for_all_robots
def check_robot_spec_save_load(id_robot, robot): #@UnusedVariable
    spec = robot.get_spec()
    spec_struct = spec.to_yaml()
    spec_struct_yaml = yaml.dump(spec_struct)
    spec_struct2 = yaml.load(spec_struct_yaml)
    spec2 = spec.from_yaml(spec_struct2)
    assert spec == spec2


