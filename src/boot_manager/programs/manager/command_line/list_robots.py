from pprint import pformat

from bootstrapping_olympics import get_conftools_robots

from .main import BOM


class CmdListRobots(BOM.get_sub()):
    '''Shows a summary of the robots in the configuration. '''

    cmd = 'list-robots'

    def define_program_options(self, params):
        params.add_flag('verbose', short='-v', help='Show more verbose output')

    def go(self):
        verbose = self.get_options().verbose

        robots = get_conftools_robots()
        which = robots.keys()  # TODO: selection, natsort

        print('I know %d robots:' % len(robots))

        max_len = max(len(x) for x in which)
        formats = '%%%ds: %%s' % (max_len + 1)
        for id_robot in which:
            robot_spec = robots[id_robot]
            print(formats % (id_robot, robot_spec['desc']))

        if verbose:
            for id_robot in which:
                robot_spec = robots[id_robot]
                print(pformat(robot_spec))
        else:
            print('Use "-v" to see more information.')
