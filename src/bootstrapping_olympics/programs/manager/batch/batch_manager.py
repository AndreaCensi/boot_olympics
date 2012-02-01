from . import BatchConfigMaster, logger
from ..meat import DataCentral
from bootstrapping_olympics.utils import (safe_makedirs, safe_symlink, UserError,
    expand_string)
from compmake import batch_command
from conf_tools import import_name
import os


try:
    import compmake #@UnusedImport
except:
    logger.error('Compmake not installed; multiprocessor '
                 'processes not available.')


def batch_process_manager(data_central, which_sets, command=None):
    from compmake import comp_prefix, use_filesystem, compmake_console

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

    if len(which_sets) == 1:
        combid = which_sets[0]
    else:
        combid = '-'.join(which_sets)

    # Create the new root        
    root = data_central.root
    root_set = os.path.join(data_central.root, 'sets', combid)
    safe_makedirs(root_set)
    data_central_set = DataCentral(root_set)

    # add symbolic links to logs and config
    safe_symlink(os.path.join(root, 'config'),
               os.path.join(root_set, 'config'))

    safe_makedirs(os.path.join(root_set, 'logs'))
    safe_symlink(os.path.join(root, 'logs'),
                 os.path.join(root_set, 'logs', 'original'))

    storage = data_central_set.get_dir_structure().get_storage_dir()
    compmake_storage = os.path.join(storage, 'compmake')
    logger.debug('Using storage %r.' % compmake_storage)
    use_filesystem(compmake_storage)

    for id_set in which_sets:
        if len(which_sets) > 1:
            comp_prefix(id_set)
        batch_set(data_central_set, id_set, batch_config.sets[x])

    if command:
        return batch_command(command)
    else:
        compmake_console()
        return 0


def batch_set(data_central, id_set, spec):
    function_name = spec['code'][0]
    args = spec['code'][1]
    function = import_name(function_name)
    args['data_central'] = data_central
    function(**args)



