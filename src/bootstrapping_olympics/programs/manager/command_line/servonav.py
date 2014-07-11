from ..meat import task_servonav
from .main import BOM


class CmdServoNav(BOM.get_sub()):
    cmd = 'servonav'

    def define_program_options(self, params):
        params.add_string('agent', short="-a")
        params.add_string('robot', short='-r')
        
        params.add_int("num_episodes",  default=10,
                      help="Number of episodes to simulate")
        params.add_flag("cumulative", 
                      help="Count already simulated episodes.")
        params.add_float("max_episode_len", default=30.0,
                      help="Maximum len of episode (seconds)")
        params.add_float("interval_print",  default=5.0,
                      help='Frequency of debug messages')
    

    def go(self):
        options = self.get_options()
        data_central = self.get_parent().get_data_central()

        id_agent = options.agent
        id_robot = options.robot
        task_servonav(data_central=data_central,
                 id_agent=id_agent,
                 id_robot=id_robot,
                 max_episode_len=options.max_episode_len,
                 num_episodes=options.num_episodes,
                 cumulative=options.cumulative,
                 interval_print=options.interval_print,
                 num_episodes_with_robot_state=options.num_episodes)  # xXX

#
# @declare_command('servonav', "servonav  -a <agent> -r <robot>")
# def cmd_task_servonav(data_central, argv):
#     '''Simulate the interaction of an agent and a robot. '''
#     parser = OptionParser(prog='servonav', usage=cmd_task_servonav.__doc__)
#     parser.disable_interspersed_args()
#     parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
#     parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
#     parser.add_option("--num_episodes", type='int', default=10,
#                       help="Number of episodes to simulate [%default]")
#     parser.add_option("--cumulative", default=False, action='store_true',
#                       help="Count already simulated episodes.")
# #    parser.add_option("--displacement", default=1.0, type='float',
# #                      help="Initial robot displacement (seconds) [%default].")
#     parser.add_option("--max_episode_len", type='float', default=30,
#                       help="Maximum len of episode (seconds) [%default]")
#     parser.add_option("--interval_print", type='float', default=5,
#                       help='Frequency of debug messages [%default]')
#     (options, args) = parser.parse_args(argv)
#
#     check_no_spurious(args)
#     check_mandatory(options, ['agent', 'robot'])
#
#     id_agent = options.agent
#     id_robot = options.robot
#     task_servonav(data_central=data_central,
#              id_agent=id_agent,
#              id_robot=id_robot,
#              max_episode_len=options.max_episode_len,
#              num_episodes=options.num_episodes,
#              cumulative=options.cumulative,
#              interval_print=options.interval_print,
#              num_episodes_with_robot_state=options.num_episodes) # xXX

