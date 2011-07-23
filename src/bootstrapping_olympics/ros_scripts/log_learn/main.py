from . import cmd_list_logs, cmd_list_states, logger, cmd_learn_log
from ...loading.load_all import load_boot_olympics_config
from optparse import OptionParser
import sys
import traceback

commands = {
    'list-logs': cmd_list_logs,
    'list-states': cmd_list_states,
    'learn-log': cmd_learn_log,
}


usage = """
    boot_log_learn --agent id_agent --robot id_robot
    
    
    List all log files:
    
        boot_log_learn [options] list-logs
        
    List all agent states: 
    
        boot_log_learn [options] list-states
        
    Learn all for an agent 
"""

def boot_log_learn(args):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-c", dest='conf_directory',
                      help="Configuration directory [%default].")
    parser.add_option("-l", dest='log_directory', default="~/.ros/log",
                      help="Log directory [%default].")
    parser.add_option("-s", dest='state_directory', default='~/boot_learning_states/',
                      help="State directory [%default].")
    (options, args) = parser.parse_args()
#
#    if options.output is None:
#        raise Exception('Please pass --output.')
    
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
        sys.exit(ret)
    except Exception as e:
        logger.error(str(e))
        logger.error(traceback.format_exc())
        sys.exit(-2) 
    
if __name__ == '__main__':
    main()
