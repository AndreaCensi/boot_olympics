from .configuration import *

from bootstrapping_olympics.programs.manager.meat import *
from bootstrapping_olympics.logs import *
from bootstrapping_olympics.agent_states import *

def jobs_comptests(context):
    from conf_tools import GlobalConfig
    GlobalConfig.global_load_dirs(['boot_manager.configs'])
        
    from . import unittests
    from comptests import jobs_registrar
    jobs_registrar(context, get_bootbatch_config())
    