''' Some example applications of agents, robots, etc. '''
from .. import logger, np, contract

__all__ = ['agents', 'robots', 'nuisances', 'rep_nuisances'] # XXX remove

from . import agents
from . import robots
from . import rep_nuisances

nuisances = rep_nuisances
