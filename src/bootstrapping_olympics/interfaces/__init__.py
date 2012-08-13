''' The software interface to the various objects. '''
from .. import logger, np, contract

BOOT_OLYMPICS_SENSEL_RESOLUTION = 'float32'

from .streamels import *
from .stream_spec import *
from .boot_spec import *
from .agent  import *
from .robot  import *
from .publisher  import *
from .observations import *
from .rep_nuisance  import *
from .live_plugin import *
