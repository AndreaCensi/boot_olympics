from . import cmd_list_logs, cmd_list_states, logger, cmd_learn_log, cmd_list_agents
from optparse import OptionParser
import sys
import traceback
import numpy as np
# XXX:
from ..agent_states import LearningStateDB
from ...configuration import BootOlympicsConfig
from bootstrapping_olympics.utils.scripts_utils import wrap_script_entry_point

commands = {
    'list-logs': cmd_list_logs,
    'list-agents': cmd_list_agents,
    'list-states': cmd_list_states,
    'learn-log': cmd_learn_log,
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
    np.seterr(all='raise')
                 
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-c", dest='conf_directory',
                      help="Configuration directory [%default].")
    parser.add_option("-l", dest='log_directory',
                      default="~/.ros/log",
                      help="Log directory [%default].")
    parser.add_option("-s", dest='state_directory',
                      default=LearningStateDB.DEFAULT_DIR,
                      help="State directory [%default].")
    (options, args) = parser.parse_args()
    
    if not args:
        msg = 'Please supply command. Available: %s' % ", ".join(commands.keys())
        raise Exception(msg)
    
    cmd = args[0]
    cmd_options = args[1:]
    
    if not cmd in commands:
        msg = ('Unknown command %r. Available: %s.' % 
               (cmd, ", ".join(commands.keys())))
        raise Exception(msg)
    
    BootOlympicsConfig.load(options.conf_directory)
     
    return commands[cmd](options, cmd_options)
    

def main(): 
    wrap_script_entry_point(boot_log_learn, logger)

if __name__ == '__main__':
    main()
