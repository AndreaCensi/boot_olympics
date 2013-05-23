from contracts import contract
from quickapp import CompmakeContext

@contract(context=CompmakeContext, episodes='list(str)')
def iterate_context_episodes(context, episodes):
    """ Yields context, id_episode,  """
    for id_episode in episodes:
        name = _good_context_name(id_episode)
        e_c = context.child(name)
        yield e_c, id_episode

@contract(context=CompmakeContext, agents='list(str)')
def iterate_context_agents(context, agents):
    """ Yields context, id_agent,  """
    for id_agent in agents:
        name = _good_context_name(id_agent)
        e_c = context.child(name)
        yield e_c, id_agent
        
@contract(context=CompmakeContext, episodes='list(str)', agents='list(str)')
def iterate_context_agents_and_episodes(context, agents, episodes):
    """ Yields context, id_agent, id_episode  """
    for cc, id_agent in iterate_context_agents(context, agents):
        for c, id_episode in iterate_context_episodes(cc, episodes):
            yield c, id_agent, id_episode

def iterate_context_robots(context, robots):
    """ Yields context, id_robot  """
    for id_robot in robots:
        name = _good_context_name(id_robot)
        e_c = context.child(name)
        yield e_c, id_robot



def _good_context_name(id_object):
    return id_object.replace('-', '')


