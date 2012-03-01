''' Some example applications of agents, robots, etc. '''
from .. import logger, np, contract
from ..utils import assign_all_to_module

__all__ = ['agents', 'robots', 'nuisances'] # XXX remove

from . import agents
from . import robots
from . import nuisances

assign_all_to_module(agents)
assign_all_to_module(nuisances)
assign_all_to_module(robots)
