
import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
    
from .configuration import *
from .logs import *
from . import hdf_support
from .programs import *


def jobs_comptests(context):
    from conf_tools import GlobalConfig
    GlobalConfig.global_load_dirs(['boot_manager.configs'])
        
    from . import unittests
    from comptests import jobs_registrar
    jobs_registrar(context, get_bootbatch_config())
    