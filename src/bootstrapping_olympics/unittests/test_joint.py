from . import for_all_pairs
from StringIO import StringIO
from bootstrapping_olympics import UnsupportedSpec
from bootstrapping_olympics.display import ReprepPublisher
from bootstrapping_olympics.programs.manager.cmd_simulate import run_simulation
import cPickle as pickle

@for_all_pairs
def check_agent_init(id_agent, agent, id_robot, robot):
    spec = robot.get_spec()
    try:
        agent.init(spec)
    except UnsupportedSpec:
        pass

@for_all_pairs
def check_small_simulation(id_agent, agent, id_robot, robot):
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return
    for _ in run_simulation(id_robot=id_robot,
                            robot=robot,
                            id_agent=id_agent,
                            agent=agent,
                            max_observations=3, max_time=100):
        pass

    state = agent.get_state()
    check_pickable(state, 'State for agent %s (%s).' % (id_agent, agent))


def check_pickable(x, desc):
    s = StringIO()
    try: 
        pickle.dump(x, s, pickle.HIGHEST_PROTOCOL)
    except:
        if isinstance(x, dict):
            for k, v in x.items():
                try: 
                    pickle.dump(v, s, pickle.HIGHEST_PROTOCOL)
                except Exception as e:
                    msg = 'Cannot dump state var %r (%s): %s' % (k, desc, e)
                    raise Exception(msg)
        else:
            raise
                    
                
@for_all_pairs
def check_publish(id_agent, agent, id_robot, robot):
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec:
        return
    
    # Check first without observations
    publisher = ReprepPublisher()   
    agent.publish(publisher)
    
    for _ in run_simulation(id_robot=id_robot,
                            robot=robot,
                            id_agent=id_agent,
                            agent=agent,
                            max_observations=10, max_time=100):
        pass

    publisher = ReprepPublisher()   
    agent.publish(publisher)
    
 
                
    
