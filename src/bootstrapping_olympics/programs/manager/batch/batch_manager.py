from . import BatchConfigMaster, logger
from bootstrapping_olympics.utils import UserError, expand_string
from compmake import comp_prefix, use_filesystem, compmake_console
from conf_tools import import_name
import os

def batch_process_manager(data_central, which_sets):
    batch_config = BatchConfigMaster()
    configs = data_central.get_dir_structure().get_config_directories()
    for config in configs:
        batch_config.load(config)
        
    sets_available = batch_config.sets.keys()
    
    logger.info('Available: %r' % sets_available)
    logger.info('Sets:      %r' % which_sets)
    which_sets = expand_string(which_sets, options=sets_available)
    
    if not which_sets:
        msg = 'Specified sets not found.'
        msg += ' Available: %s' % sets_available
        raise UserError(msg)
    
    logger.info('Expanded:  %r' % which_sets)

    for x in which_sets:
        if not x in sets_available:
            msg = 'Set %r not available.' % x
            raise UserError(msg)
        
    storage = data_central.get_dir_structure().get_storage_dir()
    if len(which_sets) == 1:    
        compmake_storage = os.path.join(storage, 'compmake', which_sets[0])
    else:
        compmake_storage = os.path.join(storage, 'compmake', 'common_storage')
    logger.debug('Using storage %r.' % compmake_storage)
    use_filesystem(compmake_storage)

    for id_set in which_sets:
        batch_set(data_central, id_set, batch_config.sets[x])
        
    compmake_console() 

        
def batch_set(data_central, id_set, spec):
    comp_prefix(id_set)
    function_name = spec['code'][0]
    args = spec['code'][1]
    function = import_name(function_name)
    args['data_central'] = data_central
    function(**args)
    
    

