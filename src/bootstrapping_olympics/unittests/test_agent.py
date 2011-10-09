from bootstrapping_olympics import AgentInterface
from . import for_all_agents

@for_all_agents
def check_agent_type(id_agent, agent):
    assert isinstance(agent, AgentInterface)
