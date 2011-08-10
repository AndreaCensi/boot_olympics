from . import cmd_list_logs, cmd_list_states, logger, cmd_learn_log, cmd_list_agents
from ...loading.load_all import load_boot_olympics_config
from optparse import OptionParser
import sys
import traceback
from bootstrapping_olympics.ros_scripts.log_learn.learning_state import LearningStateDB

commands = {
    'list-logs': cmd_list_logs,
    'list-agents': cmd_list_agents,
    'list-states': cmd_list_states,
    'learn-log': cmd_learn_log,
}

# TODO: make decorators

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
        logger.error(msg)
        return -2
    
    cmd = args[0]
    cmd_options = args[1:]
    
    if not cmd in commands:
        msg = 'Unknown command %r. Available: %s.' % (cmd, ", ".join(commands.keys()))
        logger.error(msg)
        return -3
    
    load_boot_olympics_config(options.conf_directory)
     
    return commands[cmd](options, cmd_options)
    
def main():
    try:
        ret = boot_log_learn(sys.argv[1:])
        logger.debug('Graceful exit with return code %d.' % ret)
        sys.exit(ret)
    except Exception as e:
        logger.error(str(e))
        logger.error(traceback.format_exc())
        sys.exit(-2) 
    
if __name__ == '__main__':
    main()
