from . import Storage, OptionParser, logger
from ....utils import wrap_script_entry_point, UserError, substitute
from ..meat import DataCentral, DirectoryStructure
import contracts


commands_list = "\n".join([ '  %-15s  %s\n  %-15s  Usage: %s' % 
                           (cmd, f.__doc__, '', f.short_usage) 
                           for cmd, f in Storage.commands.items()])

usage_pattern = """

    ${cmd} [options]  <command>  [command options]
    
Available commands:

${commands_list}

Use: `${cmd}  <command> -h' to get more information about that command.  
""" 

def boot_olympics_manager(args):
    usage = substitute(usage_pattern, commands_list=commands_list,
                       cmd='boot_olympics_manager')
    
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-d", dest='boot_root',
                      default=DirectoryStructure.DEFAULT_ROOT,
                      help='Root directory with logs, config, etc. [%default]')
    
    parser.add_option("--contracts", default=False, action='store_true',
                      help="Activate PyContracts checker (disbaled by default)")

#    def add_log_dir(option, opt, value, parser):
#
#    parser.add_option("-c", action="callback", callback=my_callback)

    (options, args) = parser.parse_args()
    
    if not args:
        msg = ('Please supply command. Available: %s' 
               % ", ".join(Storage.commands.keys()))
        raise UserError(msg)
    
    # TODO: option
    # np.seterr(all='raise')
    if not options.contracts:
        contracts.disable_all()
    
    # TODO: make more elegant
    try:
        from vehicles import VehiclesConfig
        VehiclesConfig.load()
        VehiclesConfig.load(options.boot_root)
    except ImportError:
        pass
        
    data_central = DataCentral(options.boot_root)
    # TODO: additional directories
    # TODO: read from environment variables    
    
    cmd = args[0]
    cmd_options = args[1:]
    
    if not cmd in Storage.commands:
        msg = ('Unknown command %r. Available: %s.' % 
               (cmd, ", ".join(Storage.commands.keys())))
        raise UserError(msg)
    
    return Storage.commands[cmd](data_central, cmd_options)
    

def manager_main(): 
    wrap_script_entry_point(boot_olympics_manager, logger)

if __name__ == '__main__':
    manager_main()
