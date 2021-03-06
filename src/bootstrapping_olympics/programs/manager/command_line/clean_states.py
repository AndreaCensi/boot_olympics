# from . import check_no_spurious, logger, OptionParser
from . import declare_command, check_no_spurious, OptionParser
from .common import check_mandatory
from bootstrapping_olympics import logger

@declare_command('clean-states', 'clean-states')
def cmd_clean_states(data_central, argv):
    ''' Cleans agents states. '''
    # TODO: to implement
    parser = OptionParser(prog='clean-states',
                          usage=cmd_clean_states.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    (options, args) = parser.parse_args(argv)
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    
    id_agent = options.agent
    id_robot = options.robot

    db = data_central.get_agent_state_db()

    # TODO: check we know agent and robot?
    
    if not db.has_state(id_agent=id_agent, id_robot=id_robot):
        logger.info('The state has already been cleaned.')
    else:
        logger.info('Cleaning state for %r/%r...' % (id_agent, id_robot))
        db.delete_state(id_agent=id_agent, id_robot=id_robot)
