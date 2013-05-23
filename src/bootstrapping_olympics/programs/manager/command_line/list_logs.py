from . import check_no_spurious, OptionParser, declare_command
from conf_tools.utils import friendly_path


@declare_command('list-logs', 'list-logs [-R] [-e] [-s] [-l]')
def cmd_list_logs(data_central, argv):
    '''Shows information about every log. '''

    parser = OptionParser(prog='list-logs', usage=cmd_list_logs.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-R", dest='refresh', action='store_true',
                      default=False,
                      help="Ignores global cache.")
    parser.add_option("-l", dest='display_logs', action='store_true',
                      default=False,
                      help="Displays all logs.")
    parser.add_option("-s", dest='display_streams', action='store_true',
                      default=False,
                      help="Displays all streams.")
    parser.add_option("-e", dest='display_episodes', action='store_true',
                      default=False, help="Displays all episodes.")
    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)

    index = data_central.get_log_index(ignore_cache=options.refresh)
    display_logs = options.display_logs 
    display_streams = options.display_streams
    display_episodes = options.display_episodes 
    
    do_list_logs(index, display_logs, display_streams, display_episodes)

def do_list_logs(index, display_logs=False, display_streams=False, display_episodes=False):
    print('Index contains %d bag files with boot data.' % 
                len(index.file2streams))
    print('In total, there are %d robots:' % len(index.robots2streams))

    for robot, streams in index.robots2streams.items():
        print('- robot %r has %d streams.' % (robot, len(streams)))
        if streams:
            total_length = 0
            total_obs = 0
            total_episodes = 0
            agents = set()
            for stream in streams:
                total_length += stream.get_length()
                total_obs += stream.get_num_observations()
                total_episodes += len(stream.get_id_episodes())
                agents.update(stream.get_id_agents())
            print('            spec: %s' % streams[0].get_spec())
            print('    total length: %.1f minutes' % 
                        (total_length / 60.0))
            print('  total episodes: %d' % (total_episodes))
            print('   total samples: %d' % (total_obs))
            print('          agents: %s' % list(agents))

        if display_logs:
            for stream in streams:
                print('   * length %5ds' % stream.get_length())
                print('     %s' % friendly_path(stream.get_filename()))

    if display_streams:
        for filename, streams in index.file2streams.items():
            if streams:
                print('In file %s:' % friendly_path(filename))

                for stream in streams:
                    print(' - there is stream: %s' % (stream))
            else:
                print('  No bootstrapping data found. ')

    if display_episodes:
        for robot, streams in index.robots2streams.items():
            print()
            print('Episodes for robot %r:' % robot)
            for stream in streams:
                print('- %s: stream %s' % (robot, stream))
                for episode in stream.get_episodes():
                    print('  contains episode: %s' % episode)

