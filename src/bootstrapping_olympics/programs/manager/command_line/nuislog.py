from . import declare_command, OptionParser, check_mandatory, check_no_spurious
from bootstrapping_olympics.programs.manager.meat.nuislog import task_nuislog


@declare_command('nuislog',
                 "nuislog -r <robot> -e <equiv_robot>")
def cmd_nuislog(data_central, argv):
    ''' Runs a log through a nuisance.

        Example:
            bom -d sets/bv1bds2 nuislog -r Se0Vrb1co -e Yrl1Se0Vrb1co
    '''
    parser = OptionParser(prog='nuislog', usage=cmd_nuislog.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-e", "--equiv", dest='equiv', help="EquivRobot ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
 
    (options, args) = parser.parse_args(argv)

    check_no_spurious(args)
    check_mandatory(options, ['equiv', 'robot'])

    id_equiv = options.equiv
    id_robot = options.robot
    task_nuislog(data_central, id_equiv=id_equiv, id_robot=id_robot)
