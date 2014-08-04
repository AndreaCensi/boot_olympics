from bootstrapping_olympics import get_conftools_agents, get_conftools_robots
from contracts import contract
from streamels.exceptions import UnsupportedSpec
import numpy as np

def get_tranches(ids, episodes_per_tranche=10):
    """ Returns a list of list """
    l = []
    num_tranches = int(np.ceil(len(ids) * 1.0 / episodes_per_tranche))
    for t in range(num_tranches):
        e_from = t * episodes_per_tranche
        e_to = min(len(ids), e_from + episodes_per_tranche)
        l.append([ids[i] for i in range(e_from, e_to)])
    return l


@contract(id_agent='str', K='int')
def episode_id_exploration(id_agent, K):
    return 'ep_expl_%s_%05d' % (id_agent, K)

@contract(id_agent='str', K='int')
def episode_id_servoing(id_agent, K):
    return 'ep_serv_%s_%05d' % (id_agent, K)

@contract(id_agent='str', K='int')
def episode_id_servonav(id_agent, K):
    return 'ep_servonav_%s_%05d' % (id_agent, K)

def are_compatible(robot, agent):
    try:
        agent.init(robot.get_spec())
    except UnsupportedSpec as e:
        return False, str(e)
    
    return True, None

def instance_agent(id_agent):
    return get_conftools_agents().instance(id_agent)

def instance_robot(id_robot):
    return get_conftools_robots().instance(id_robot)

