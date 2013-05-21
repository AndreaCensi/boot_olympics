''' Some example applications of agents, robots, etc. '''
from .. import logger, np, contract
from ..utils import assign_all_to_module

__all__ = ['agents', 'robots', 'nuisances', 'nuisances_causal', 'live_plugins']

from . import agents
from . import robots
from . import nuisances
from . import nuisances_causal
from . import live_plugins

# Useful for documentation purposes (but not used so far)
assign_all_to_module(agents)
assign_all_to_module(nuisances)
assign_all_to_module(nuisances_causal)
assign_all_to_module(robots)
assign_all_to_module(live_plugins)
