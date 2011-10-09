from bootstrapping_olympics import BootOlympicsConfig, Constants
import os
from . import logger

def make_sure_loaded():
    if not BootOlympicsConfig.loaded:
        # TODO: add from environment variable
        v = Constants.TEST_ADDITIONAL_CONFIG_DIR_ENV
        if v in os.environ:
            for dirname in os.environ[v].split(':'):
                logger.info('Using additional dir %r' % dirname)
                BootOlympicsConfig.load(dirname)             
        else:
            logger.info('Use env var %s to add more config dirs.' % v)
            BootOlympicsConfig.load() 


def all_robots():
    ''' Returns a list of all robots IDs. '''
    make_sure_loaded()
    robots = list(BootOlympicsConfig.robots.keys())
    if not robots:
        raise Exception('No robots defined in this configuration.')
    return robots

def all_agents():
    ''' Returns a list of all agents IDs. '''
    make_sure_loaded()
    agents = list(BootOlympicsConfig.agents.keys())
    if not agents:
        raise Exception('No agents defined in this configuration.')
    return agents
    
def get_robot(id_robot):
    make_sure_loaded()
    return BootOlympicsConfig.robots.instance(id_robot) #@UndefinedVariable

def get_agent(id_agent):
    make_sure_loaded()
    agent = BootOlympicsConfig.agents.instance(id_agent) #@UndefinedVariable
    logger.debug('Instantiating %s = %s' % (id_agent, agent))
    return agent
