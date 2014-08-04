''' Some example applications of agents, robots, etc. '''

__all__ = [
    'agents', 
    'robots', 
    'nuisances', 
    'nuisances_causal',
]

from . import agents
from . import robots
from . import nuisances
from . import nuisances_causal

# Useful for documentation purposes (but not used so far)
from ..utils import assign_all_to_module
assign_all_to_module(agents)
assign_all_to_module(nuisances)
assign_all_to_module(nuisances_causal)
assign_all_to_module(robots)
