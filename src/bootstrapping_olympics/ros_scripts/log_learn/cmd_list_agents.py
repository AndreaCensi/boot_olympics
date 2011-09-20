from . import logger
from optparse import OptionParser 
from bootstrapping_olympics import BootOlympicsConfig
from pprint import pformat

__all__ = ['cmd_list_agents']

def cmd_list_agents(main_options, argv):
    '''Shows a summary of the agents in the configuration. '''
    parser = OptionParser(usage=cmd_list_agents.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-v", dest='verbose', default=False, action='store_true',
                      help="Show more verbose output.") 
    (options, args) = parser.parse_args(argv)    
    
    if args: 
        msg = ('Spurious args (%s)' % args)
        raise Exception(msg)
     
    agents = BootOlympicsConfig.agents
    which = BootOlympicsConfig.agents.keys() # TODO: selection
    
    logger.info('I know %d agents:' % len(agents))
    for id_agent in which:
        agent_spec = agents[id_agent]
        logger.info('%25s: %s' % (id_agent, agent_spec['desc']))
    
    if options.verbose:
        for id_agent in which:
            agent_spec = agents[id_agent]
            logger.info(pformat(agent_spec))
    else:
        logger.info('Use "-v" to see more information.')
         

cmd_list_agents.short_usage = 'list-agents [-v]'
    
