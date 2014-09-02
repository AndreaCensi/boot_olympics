
import logging
logging.basicConfig()
logger = logging.getLogger(__name__)
    
from .configuration import *
from .logs import *
from . import hdf_support
from .programs import *


def jobs_comptests(context):
    from conf_tools import GlobalConfig
    config_dirs = [
        'bootstrapping_olympics.configs',
        'boot_manager.configs',
    ]
    GlobalConfig.global_load_dirs(config_dirs)
        
    from . import unittests
    from comptests import jobs_registrar
    jobs_registrar(context, get_bootbatch_config())
    