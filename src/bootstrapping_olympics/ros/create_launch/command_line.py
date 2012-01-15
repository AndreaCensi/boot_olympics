from ...programs.manager.command_line import (logger, OptionParser,
    check_no_spurious, declare_command)
from ...utils import check_contained, isodate, make_sure_dir_exists
from ..launch_xml import (wrap_python_robot, wrap_python_agent,
    create_launch_xml)
import itertools
import os


@declare_command('create-launch',
                 'create-launch -a <agent> -r <robot> ')
def cmd_create_launch(data_central, argv):
    '''Creates ROS launch files for all combinations of agents and robots.'''
    parser = OptionParser(prog='create-launch',
                          usage=cmd_create_launch.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='id_agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='id_robot', help="Robot ID")

    parser.add_option('-p', '--package', dest='package',
                      default='bootstrapping_adapter',
            help='Which ROS package to put the launch file in [%default].')

    # TODO: bag
    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)

    bo_config = data_central.get_bo_config()

    if options.id_agent is None:
        id_agents = bo_config.agents.keys()
    else:
        id_agents = [options.id_agent] # TODO: expand

    if options.id_robot is None:
        id_robots = bo_config.robots.keys()
    else:
        id_robots = [options.id_robot] # TODO: expand

    for id_agent in id_agents:
        check_contained(id_agent, bo_config.agents)
    for id_robot in id_robots:
        check_contained(id_robot, bo_config.robots)

    from roslib import packages #@UnresolvedImport
    out = packages.get_pkg_dir(options.package)
    outdir = os.path.join(out, 'launch', 'boot_olympics')

    root = os.path.realpath(data_central.root)
    for id_robot, id_agent in itertools.product(id_robots, id_agents):
        robot_ros_node = wrap_python_robot(bo_config.robots[id_robot], root)
        agent_ros_node = wrap_python_agent(bo_config.agents[id_agent], root)
        xml = create_launch_xml(agent_ros_node=agent_ros_node,
                                robot_ros_node=robot_ros_node,
                                agent_node_id='my_agent',
                                robot_node_id='my_robot',
                                namespace='boot_olympics',
                                bag=None,
                                output=None)
        basename = '%s-%s' % (id_robot, id_agent)

        filename = os.path.join(outdir, '%s.launch' % basename)

        make_sure_dir_exists(filename)
        with open(filename, 'w') as f:
            f.write('<!-- Created by %s on %s -->\n' %
                    ('boot_olympics', isodate()))
            f.write(xml)

        logger.info('Writing on %r.' % filename)



