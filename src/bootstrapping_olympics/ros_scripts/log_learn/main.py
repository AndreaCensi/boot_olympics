from . import (cmd_list_logs, cmd_simulate, cmd_list_states, logger,
    cmd_learn_log, cmd_list_agents, cmd_list_robots, DataCentral)
from ...configuration import DirectoryStructure
from ...utils import wrap_script_entry_point, UserError
from optparse import OptionParser
import numpy as np
import contracts

commands = {
    'list-logs': cmd_list_logs,
    'list-agents': cmd_list_agents,
    'list-robots': cmd_list_robots,
    'list-states': cmd_list_states,
    'learn-log': cmd_learn_log,
    'simulate': cmd_simulate
}


commands_list = "\n".join([ '  %-15s  %s\n  %-15s  Usage: %s' % 
                           (cmd, f.__doc__, '', f.short_usage) 
                           for cmd, f in commands.items()])

usage = """

    boot_learn [options]  <command>  [command options]
    
Available commands:

%s

Use: `boot_learn  <command> -h' to get more information about that command.  
""" % commands_list


def boot_log_learn(args):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-d", dest='boot_root',
                      default=DirectoryStructure.DEFAULT_ROOT,
                      help='Root directory with logs, config, etc. [%default]')
    
    parser.add_option("--contracts", default=False, action='store_true',
                      help="Activate PyContracts checker (disbaled by default)")

    (options, args) = parser.parse_args()
    
    if not args:
        msg = 'Please supply command. Available: %s' % ", ".join(commands.keys())
        raise UserError(msg)
    
    # TODO: option
    np.seterr(all='raise')
    if not options.contracts:
        contracts.disable_all()
    
    
    data_central = DataCentral(options.boot_root)
    # TODO: additional directories
    # TODO: read from environment variables    
    
    cmd = args[0]
    cmd_options = args[1:]
    
    if not cmd in commands:
        msg = ('Unknown command %r. Available: %s.' % 
               (cmd, ", ".join(commands.keys())))
        raise Exception(msg)
    
    return commands[cmd](data_central, cmd_options)
    

def main(): 
    wrap_script_entry_point(boot_log_learn, logger)

if __name__ == '__main__':
    main()
