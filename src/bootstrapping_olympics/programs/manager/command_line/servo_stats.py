from . import OptionParser, check_no_spurious, check_mandatory, declare_command
from ..meat import servo_stats_summaries, servo_stats_report
 
@declare_command('servo_stats', "servo_stats  -a <agent> -r <robot>")
def cmd_servo_stats(data_central, argv):
    '''Simulate the interaction of an agent and a robot. ''' 
    parser = OptionParser(usage=cmd_servo_stats.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    id_agent = options.agent
    id_robot = options.robot
    
    summaries = servo_stats_summaries(data_central, id_agent, id_robot)    
    servo_stats_report(data_central, id_agent, id_robot, summaries)
