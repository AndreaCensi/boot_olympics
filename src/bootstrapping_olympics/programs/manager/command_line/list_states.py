from . import logger, check_no_spurious, OptionParser, declare_command
from contracts import describe_value

@declare_command('list-states', 'list-states [-v]')
def cmd_list_states(data_central, argv):
    '''Shows a summary of the states present in DB. '''
    parser = OptionParser(usage=cmd_list_states.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-v", dest='verbose', default=False, action='store_true',
                      help="Show more verbose output.") 
    (options, args) = parser.parse_args(argv)    
    
    check_no_spurious(args)
    db = data_central.get_agent_state_db()
    
    combinations = list(db.list_states())
    if not combinations:
        logger.info('No learning states saved in DB.')
    else:
        logger.info('Found %d combinations in DB.' % len(combinations))
        
    for id_agent, id_robot in combinations:
        logger.info('- Found state a: %-35s  r: %-25s' % (id_agent, id_robot))
        
        if options.verbose:
            try:
                state = db.get_state(id_robot=id_robot, id_agent=id_agent)
                logger.info('  # episodes: %s' % len(state.id_episodes))
                logger.info('      object: %s' % describe_value(state.agent_state))
            except Exception as e:
                logger.error('  (could not load state: %s) ' % e)
             
    if not options.verbose:
        logger.debug('Use -v for more information.') 
        
        
