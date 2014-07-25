from ..meat import DataCentral
from boot_manager import get_conftools_bootbatchsets
from bootstrapping_olympics import logger
from bootstrapping_olympics.utils import safe_makedirs, safe_symlink
from conf_tools import ConfToolsException, import_name
from pprint import pformat
from quickapp import iterate_context_names
import os


def batch_process_manager(context, data_central, which_sets):
#     
#     batch_config = BatchConfigMaster()
#     configs = data_central.get_dir_structure().get_config_directories()
#     for config in configs:
#         batch_config.load(config)
# 
#     
    sets_config = get_conftools_bootbatchsets()
#     sets_config =  batch_config.sets 

    which_sets_int = sets_config.expand_names(which_sets) 

    for x in which_sets_int:
        sets_config[x] 
        
    if len(which_sets_int) == 1:
        combid = which_sets[0]
    else:
        combid = '-'.join(which_sets)

    # Create the new root        
    root = data_central.root
    root_set = os.path.join(data_central.root, 'sets', combid)
    safe_makedirs(root_set)
    data_central_set = DataCentral(root_set)

    # add symbolic links to logs and config
    main_config = os.path.realpath(os.path.join(root, 'config'))
    set_config = os.path.join(root_set, 'config')
    safe_symlink(main_config, set_config) 

    safe_makedirs(os.path.join(root_set, 'logs'))
    safe_symlink(os.path.join(root, 'logs'),
                 os.path.join(root_set, 'logs', 'original'))

    print('id_sets: %s' % which_sets_int)
    for c, id_set in iterate_context_names(context, which_sets_int):
        print('set %r'% id_set)
        try:
            spec = sets_config[x]
            batch_set(c, data_central_set, id_set, spec)
        except ConfToolsException:
            msg = ('Bad configuration for the set %r with spec\n %s' % 
                   (id_set, pformat(spec)))
            logger.error(msg)
            raise 

def batch_set(context, data_central, id_set, spec):  # @UnusedVariable
    function_name = spec['code'][0]
    args = spec['code'][1]
    function = import_name(function_name)
    function(context=context, data_central=data_central, **args)



