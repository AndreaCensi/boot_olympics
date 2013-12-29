import contracts

from bootstrapping_olympics import LogsFormat, BootOlympicsConstants, logger
from bootstrapping_olympics.utils import (wrap_script_entry_point, UserError,
    substitute)
from conf_tools import ConfToolsException, GlobalConfig
import numpy as np

from . import Storage, OptionParser
from ..meat import DataCentral, DirectoryStructure


commands_list = "\n".join(['  %-15s  %s\n  %-15s  Usage: %s' % 
                           (cmd, f.__doc__, '', f.short_usage)
                           for cmd, f in Storage.commands.items()])

usage_pattern = """

    ${cmd} [options]  <command>  [command options]
    
Available commands:

${commands_list}

Use: `${cmd}  <command> -h' to get more information about that command.  
"""


def boot_olympics_manager(arguments):
    usage = substitute(usage_pattern, commands_list=commands_list,
                       cmd='boot_olympics_manager')

    parser = OptionParser(prog='boot_olympics_manager', usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-d", dest='boot_root', default=None,
                      help='Root directory with logs, config, etc. [%default]')

    parser.add_option("-c", dest='extra_conf_dirs', action="append", default=[],
                      help='Adds an extra config dir.')

    parser.add_option("-l", dest='extra_log_dirs', action="append", default=[],
                      help='Adds an extra directory storing logs.')

    parser.add_option("--contracts", default=False, action='store_true',
                      help="Activate PyContracts (disabled by default)")

    parser.add_option("--seterr", dest='seterr', default="warn",
                      help="Sets np.seterr. "
                      "Possible values: ignore, warn, raise, print, log")

    parser.add_option("--profile", default=False, action='store_true',
                      help="Use Python profiler")

    available = LogsFormat.formats.keys()
    parser.add_option("--logformat", dest='log_format',
                      default=BootOlympicsConstants.DEFAULT_LOG_FORMAT,
                      help="Choose format for writing logs in %s. [%%default]"
                        % str(available))

    (options, args) = parser.parse_args(arguments)

    if not args:
        msg = ('Please supply command. Available: %s'
               % ", ".join(Storage.commands.keys()))
        raise UserError(msg)

    cmd = args[0]
    cmd_options = args[1:]

    if not cmd in Storage.commands:
        msg = ('Unknown command %r. Available: %s.' % 
               (cmd, ", ".join(Storage.commands.keys())))
        raise UserError(msg)

    np.seterr(all=options.seterr)
    # underflow is very common in all libraries (e.g. matplotlib)
    np.seterr(under='warn')

    if not options.contracts:
        contracts.disable_all()

    if options.boot_root is None:
        options.boot_root = DirectoryStructure.DEFAULT_ROOT
        logger.info('Using %r as default root directory '
                    '(use -d <dir> to change)' % options.boot_root)


    data_central = DataCentral(options.boot_root)
    
    GlobalConfig.global_load_dir('default')  # need skins
    for dirname in options.extra_conf_dirs:
        GlobalConfig.global_load_dir(dirname)
    
    dir_structure = data_central.get_dir_structure() 
    dir_structure.set_log_format(options.log_format)
    for dirname in options.extra_log_dirs:
        dir_structure.add_log_directory(dirname)

    def go():
        return Storage.commands[cmd](data_central, cmd_options)

    if not options.profile:
        go()
    else:
        logger.warning('Note: the profiler does not work when using '
                       'parallel execution. (use "make" instead of "parmake").')
        import cProfile
        cProfile.runctx('go()', globals(), locals(), 'bom_prof')
        import pstats
        p = pstats.Stats('bom_prof')
        p.sort_stats('cumulative').print_stats(30)
        p.sort_stats('time').print_stats(30)


def manager_main():
    exceptions_no_traceback = (UserError, ConfToolsException)
    return wrap_script_entry_point(boot_olympics_manager, logger,
                        exceptions_no_traceback=exceptions_no_traceback)


if __name__ == '__main__':
    manager_main()
