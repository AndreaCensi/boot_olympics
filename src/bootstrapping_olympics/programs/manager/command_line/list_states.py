
from contracts import describe_value

from .main import BOM


class CmdListStates(BOM.get_sub()):
    '''Shows a summary of the states present in DB. '''

    cmd = 'list-states'

    def define_program_options(self, params):
        params.add_flag('verbose', short='-v', help='Show more verbose output')

    def go(self):
        verbose = self.get_options().verbose

        data_central = self.get_parent().get_data_central()
        db = data_central.get_agent_state_db()
    
        combinations = list(db.list_states())
        if not combinations:
            self.info('No learning states saved in DB.')
        else:
            self.info('Found %d combinations in DB.' % len(combinations))
    
        for id_agent, id_robot in combinations:
            self.info('- Found state a: %-35s  r: %-25s' % (id_agent, id_robot))
    
            if verbose:
                try:
                    state = db.get_state(id_robot=id_robot, id_agent=id_agent)
                    self.info('  # episodes: %s' % len(state.id_episodes))
                    self.info('      object: %s' %
                                describe_value(state.agent_state))
                except Exception as e:
                    self.error('  (could not load state: %s) ' % e)
    
        if not verbose:
            self.debug('Use -v for more information.')
#
#
# @declare_command('list-states', 'list-states [-v]')
# def cmd_list_states(data_central, argv):
#     parser = OptionParser(prog='list-states',
#                           usage=cmd_list_states.short_usage)
#     parser.disable_interspersed_args()
#     parser.add_option("-v", dest='verbose', default=False, action='store_true',
#                       help="Show more verbose output.")
#     (options, args) = parser.parse_args(argv)
#
#     check_no_spurious(args)
#     db = data_central.get_agent_state_db()
#
#     combinations = list(db.list_states())
#     if not combinations:
#         logger.info('No learning states saved in DB.')
#     else:
#         logger.info('Found %d combinations in DB.' % len(combinations))
#
#     for id_agent, id_robot in combinations:
#         logger.info('- Found state a: %-35s  r: %-25s' % (id_agent, id_robot))
#
#         if options.verbose:
#             try:
#                 state = db.get_state(id_robot=id_robot, id_agent=id_agent)
#                 logger.info('  # episodes: %s' % len(state.id_episodes))
#                 logger.info('      object: %s' %
#                             describe_value(state.agent_state))
#             except Exception as e:
#                 logger.error('  (could not load state: %s) ' % e)
#
#     if not options.verbose:
#         logger.debug('Use -v for more information.')


