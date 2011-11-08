from . import OptionParser, check_no_spurious, check_mandatory, declare_command
from ..meat import task_servo
 
@declare_command('servo', "servo  -a <agent> -r <robot>")
def cmd_task_servo(data_central, argv):
    '''Simulate the interaction of an agent and a robot. ''' 
    parser = OptionParser(usage=cmd_task_servo.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--num_episodes", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--cumulative", default=False, action='store_true',
                      help="Count already simulated episodes towards the count.")
    parser.add_option("--max_episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--interval_print", type='float', default=5,
                      help='Frequency of debug messages.')
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    id_agent = options.agent
    id_robot = options.robot
    task_servo(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             max_episode_len=options.max_episode_len,
             num_episodes=options.num_episodes,
             cumulative=options.cumulative,
             interval_print=options.interval_print,
             num_episodes_with_robot_state=options.num_episodes) # xXX


