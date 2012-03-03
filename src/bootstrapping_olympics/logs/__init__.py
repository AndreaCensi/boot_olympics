''' Abstractions for reading and writing logs in various formats. '''

from .. import np, contract, getLogger


logger = getLogger(__name__)

from .boot_stream import *
from .logs_format import *
from .log_index import *

