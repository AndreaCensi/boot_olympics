from . import logger
from bootstrapping_olympics import get_boot_config
import os


def get_test_boot_config():
    boot_config = get_boot_config()
    if not boot_config.loaded:
        from pkg_resources import resource_filename  # @UnresolvedImport
        dirname = resource_filename("bootstrapping_olympics", "configs")
        dirname = os.path.join(dirname, 'for_testing')
        logger.info('Configuration not set by environment variable; loading '
                    'default testing config from %s.' % dirname)

        boot_config.load(dirname)
    return boot_config


def all_nuisances():
    boot_config = get_test_boot_config()
    nuisances = list(boot_config.nuisances.keys())  # @UndefinedVariable
    if not nuisances:
        raise Exception('No nuisances defined in this configuration.')
    return nuisances


def all_robots():
    ''' Returns a list of all robots IDs. '''
    boot_config = get_test_boot_config()
    robots = list(boot_config.robots.keys())  # @UndefinedVariable
    if not robots:
        raise Exception('No robots defined in this configuration.')
    return robots


def all_agents():
    ''' Returns a list of all agents IDs. '''
    boot_config = get_test_boot_config()
    agents = list(boot_config.agents.keys())  # @UndefinedVariable
    if not agents:
        raise Exception('No agents defined in this configuration.')
    return agents


def get_robot(id_robot):
    boot_config = get_test_boot_config()
    return boot_config.robots.instance(id_robot)  # @UndefinedVariable


def get_nuisance(id_nuisance):
    boot_config = get_test_boot_config()
    return boot_config.nuisances.instance(id_nuisance)


def get_agent(id_agent):
    boot_config = get_test_boot_config()
    agent = boot_config.agents.instance(id_agent)  # @UndefinedVariable
    logger.debug('Instantiating %s = %s' % (id_agent, agent))
    return agent
