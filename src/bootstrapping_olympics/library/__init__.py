''' Some example applications of agents, robots, etc. '''
from .. import logger, np, contract
from ..utils import assign_all_to_module

__all__ = ['agents', 'robots', 'nuisances']

from . import agents
from . import robots
from . import nuisances

# Useful for documentation purposes (but not used so far)
assign_all_to_module(agents)
assign_all_to_module(nuisances)
assign_all_to_module(robots)
