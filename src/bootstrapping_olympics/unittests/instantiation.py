from . import logger
from bootstrapping_olympics import BootOlympicsConfig
import os


def make_sure_loaded():
    if not BootOlympicsConfig.loaded:
        from pkg_resources import resource_filename #@UnresolvedImport
        dirname = resource_filename("bootstrapping_olympics", "configs")
        dirname = os.path.join(dirname, 'for_testing')
        logger.info('Configuration not set by environment variable; loading '
                    'default testing config from %s.' % dirname)

        BootOlympicsConfig.load(dirname)


def all_nuisances():
    make_sure_loaded()
    nuisances = list(BootOlympicsConfig.nuisances.keys()) #@UndefinedVariable
    if not nuisances:
        raise Exception('No nuisances defined in this configuration.')
    return nuisances


def all_robots():
    ''' Returns a list of all robots IDs. '''
    make_sure_loaded()
    robots = list(BootOlympicsConfig.robots.keys()) #@UndefinedVariable
    if not robots:
        raise Exception('No robots defined in this configuration.')
    return robots


def all_agents():
    ''' Returns a list of all agents IDs. '''
    make_sure_loaded()
    agents = list(BootOlympicsConfig.agents.keys()) #@UndefinedVariable
    if not agents:
        raise Exception('No agents defined in this configuration.')
    return agents


def get_robot(id_robot):
    make_sure_loaded()
    return BootOlympicsConfig.robots.instance(id_robot) #@UndefinedVariable


def get_nuisance(id_nuisance):
    make_sure_loaded()
    return BootOlympicsConfig.specs['nuisances'].instance(id_nuisance)


def get_agent(id_agent):
    make_sure_loaded()
    agent = BootOlympicsConfig.agents.instance(id_agent) #@UndefinedVariable
    logger.debug('Instantiating %s = %s' % (id_agent, agent))
    return agent
