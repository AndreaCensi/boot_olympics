from . import for_all_agents
from bootstrapping_olympics import AgentInterface


@for_all_agents
def check_agent_type(id_agent, agent): #@UnusedVariable
    assert isinstance(agent, AgentInterface)
