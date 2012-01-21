from . import check_no_spurious, logger, OptionParser, declare_command


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

    logger.info('Index contains %d bag files with boot data.' %
                len(index.file2streams))
    logger.info('In total, there are %d robots:'
                % len(index.robots2streams))

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
            print('    total length: %.1f minutes' %
                        (total_length / 60.0))
            print('  total episodes: %d' % (total_episodes))
            print('   total samples: %d' % (total_obs))
            print('          agents: %s' % list(agents))

        if options.display_logs:
            for stream in streams:
                print('- %5ds %s' % (stream.get_length(),
                                     stream.get_filename()))

    if options.display_streams:
        for filename, streams in index.file2streams.items():
            if streams:
                logger.info('%s' % filename)

                for stream in streams:
                    logger.info(' %s' % (stream))
            else:
                print('  No bootstrapping data found. ')

    if options.display_episodes:
        for robot, streams in index.robots2streams.items():
            for stream in streams:
                print('%s: %s' % (robot, stream))
                for episode in stream.get_episodes():
                    print('- %s' % episode)

