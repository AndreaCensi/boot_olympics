from . import declare_command, OptionParser, check_mandatory, check_no_spurious
from ..meat import task_predict
from bootstrapping_olympics.programs.manager.meat.predict import predict_report


@declare_command('predict',
                 "predict  -a <agent> -r <robot> ")
def cmd_task_predict(data_central, argv):
    '''Simulate the interaction of an agent and a robot. '''
    parser = OptionParser(prog='predict', usage=cmd_task_predict.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")

    parser.add_option("--interval_print", type='float', default=5,
                      help='Frequency of debug messages.')

    parser.add_option("--plugin", default=[],
                      action="append", type="string",
                      help="Run the specified plugin model during "
                           "learning. (eg, visualization)")

    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])

    id_agent = options.agent
    id_robot = options.robot
    stats = task_predict(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             live_plugins=options.plugin)

    predict_report(data_central, id_agent, id_robot, stats)
