from bootstrapping_olympics import BasicAgent
from bootstrapping_olympics.unittests import for_all_agents


@for_all_agents
def check_agent_type(id_agent, agent):  # @UnusedVariable
    assert isinstance(agent, BasicAgent)
