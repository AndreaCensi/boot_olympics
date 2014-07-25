from conf_tools import GlobalConfig
import os

def jobs_comptests(context):
    from . import unittests
    from comptests import jobs_registrar

    configs = []
    # get testing configuration directory 
    from pkg_resources import resource_filename  # @UnresolvedImport
    configs.append(resource_filename("boot_manager", "configs"))
    
    # load into bootstrapping_olympics
    from bootstrapping_olympics import get_boot_config
    # need to load it
    from bootstrapping_olympics.configuration.batch_config import get_bootbatch_config

    for d in configs:
        d = os.path.abspath(d)
        if not os.path.exists(d):
            raise ValueError('directory does not exist: %r' % d)
        
        GlobalConfig.global_load_dir(d) 
        
    jobs_registrar(context, get_bootbatch_config())
    