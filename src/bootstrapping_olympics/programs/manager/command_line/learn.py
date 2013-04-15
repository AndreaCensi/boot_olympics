from . import declare_command, logger, OptionParser
from . import check_no_spurious, check_mandatory
from ..meat import learn_log


@declare_command('learn-log', 'learn-log -a <AGENT> -r <ROBOT>'
                              '  [--reset] [--publish interval] [--once]')
def cmd_learn_log(data_central, argv):
    '''
        Runs the learning for a given agent and log. 
       
       
        Running a live plugin:
        
            bom  [agent/robot options] --reset --dontsave --plugin dummy
        
    '''
    parser = OptionParser(prog='learn-log', usage=cmd_learn_log.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--reset", default=False, action='store_true',
                      help="Do not use cached state.")
    parser.add_option("-p", "--publish", dest='publish_interval', type='float',
                      default=None,
                      help="Publish debug information every N cycles.")
    parser.add_option("--once", default=False, action='store_true',
                      help="Just plot the published information and exit.")
    parser.add_option("--interval_save", type='int', default=300,
                      help="Interval for saving state (seconds) [%default]")
    parser.add_option("--interval_print", type='int', default=5,
                      help="Interval for printing stats (seconds) [%default]")
    parser.add_option("--dontsave", default=False, action='store_true',
                      help="Do not save the state of the agent.")

    parser.add_option("--plugin", default=[],
                      action="append", type="string",
                      help="Run the specified plugin model during "
                           "learning. (eg, visualization)")

    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])

    if options.publish_interval is  None and not options.once:
        msg = 'Not creating any report; pass -p <interval> or --once to do it.'
        logger.info(msg)

    learn_log(data_central=data_central,
              id_agent=options.agent,
              id_robot=options.robot,
              reset=options.reset,
              publish_interval=options.publish_interval,
              publish_once=options.once,
              interval_save=options.interval_save,
              interval_print=options.interval_print,
              save_state=not(options.dontsave),
              live_plugins=options.plugin)


