'''
    A streamel (stream element) describes the *format* of the data stream.
    It specifies what are the valid values that the data can take. 

'''

import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

from .exceptions import *
from .base import *
from .streamels_checks import *
from .stream_spec import *
from .streamels_make import *
from .boot_spec import *
