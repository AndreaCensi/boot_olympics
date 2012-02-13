from . import check_no_spurious, OptionParser, declare_command
from pprint import pformat


@declare_command('list-robots', 'list-robots [-v]')
def cmd_list_robots(data_central, argv):
    '''Shows a summary of the robots in the configuration. '''
    parser = OptionParser(prog='list-robots',
                          usage=cmd_list_robots.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-v", dest='verbose',
                      default=False, action='store_true',
                      help="Show more verbose output.")
    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)

    bo_config = data_central.get_bo_config()
    robots = bo_config.robots
    which = robots.keys() # TODO: selection, natsort

    print('I know %d robots:' % len(robots))

    max_len = max(len(x) for x in which)
    formats = '%%%ds: %%s' % (max_len + 1)
    for id_robot in which:
        robot_spec = robots[id_robot]
        print(formats % (id_robot, robot_spec['desc']))

    if options.verbose:
        for id_robot in which:
            robot_spec = robots[id_robot]
            print(pformat(robot_spec))
    else:
        print('Use "-v" to see more information.')



