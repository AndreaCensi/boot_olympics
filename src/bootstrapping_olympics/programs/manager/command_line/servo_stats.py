from ..meat import servo_stats_summaries, servo_stats_report
from .main import BOM


class CmdServoStats(BOM.get_sub()):
    cmd = 'servo_stats'

    def define_program_options(self, params):
        params.add_string('agent', short="-a")
        params.add_string('robot', short='-r')

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()

        id_agent = options.agent
        id_robot = options.robot

        summaries = servo_stats_summaries(data_central, id_agent, id_robot)
        servo_stats_report(data_central, id_agent, id_robot, summaries)
#
#
# @declare_command('servo_stats', "servo_stats  -a <agent> -r <robot>")
# def cmd_servo_stats(data_central, argv):
#     parser = OptionParser(prog='servo_stats', usage=cmd_servo_stats.__doc__)
#     parser.disable_interspersed_args()
#     parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
#     parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
#     (options, args) = parser.parse_args(argv)
#
#     check_no_spurious(args)
#     check_mandatory(options, ['agent', 'robot'])
#
#     id_agent = options.agent
#     id_robot = options.robot
#
#     summaries = servo_stats_summaries(data_central, id_agent, id_robot)
#     servo_stats_report(data_central, id_agent, id_robot, summaries)
