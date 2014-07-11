from pprint import pformat

from bootstrapping_olympics import logger, get_conftools_agents

from .main import BOM


class CmdListRobots(BOM.get_sub()):
    '''Shows a summary of the agents in the configuration. '''

    cmd = 'list-agents'

    def define_program_options(self, params):
        params.add_flag('verbose', short='-v', help='Show more verbose output')

    def go(self):
        verbose = self.get_options().verbose

        agents = get_conftools_agents()
        which = agents.keys()  # TODO: selection, natsort

        print('I know %d agents:' % len(agents))

        max_len = max(len(x) for x in which)
        formats = '%%%ds: %%s' % (max_len + 1)

        self.info('I know %d agents:' % len(agents))
        for id_agent in which:
            agent_spec = agents[id_agent]
            logger.info(formats % (id_agent, agent_spec['desc']))

        if verbose:
            for id_agent in which:
                agent_spec = agents[id_agent]
                logger.info(pformat(agent_spec))
        else:
            self.info('Use "-v" to see more information.')
#
#
# @declare_command('list-agents', 'list-agents [-v]')
# def cmd_list_agents(data_central, argv):
#     '''Shows a summary of the agents in the configuration. '''
#     parser = OptionParser(prog='list-agents',
#                           usage=cmd_list_agents.short_usage)
#     parser.disable_interspersed_args()
#     parser.add_option("-v", dest='verbose', default=False, action='store_true',
#                       help="Show more verbose output.")
#     (options, args) = parser.parse_args(argv)
#
#     check_no_spurious(args)
#
#     bo_config = data_central.get_bo_config()
#     agents = bo_config.agents
#     which = bo_config.agents.keys()  # TODO: selection
#
#     max_len = max(len(x) for x in which)
#     formats = '%%%ds: %%s' % (max_len + 1)
#
#     logger.info('I know %d agents:' % len(agents))
#     for id_agent in which:
#         agent_spec = agents[id_agent]
#         logger.info(formats % (id_agent, agent_spec['desc']))
#
#     if options.verbose:
#         for id_agent in which:
#             agent_spec = agents[id_agent]
#             logger.info(pformat(agent_spec))
#     else:
#         logger.info('Use "-v" to see more information.')


