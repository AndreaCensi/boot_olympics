from simple_vehicles.loading.utils import load_configuration_entries
from bootstrapping_olympics import logger

class Configuration:
    tasks = {}
    robots = {}
    agents = {}
    events = {}


def load_configuration(directory=None,
                       pattern_tasks='*.tasks.yaml',
                       pattern_robots='*.robots.yaml',
                       pattern_agents='*.agents.yaml',
                       pattern_events='*.events.yaml'):
    ''' 
        Loads all configuration files from the directory. 
        If directory is not specified, it uses the default directory. 
    '''
    Configuration.loaded = True
    
    if directory is None:
        from pkg_resources import resource_filename #@UnresolvedImport
        directory = resource_filename("bootstrapping_olympics", "configs")
        
    logger.info('Loading configuration from %r' % directory)

    def merge(original, new):
        for x in new:
            if x in original:
                msg = ('Entry %r (%r) already present in %r.' % 
                       (x, new[x]['filename'], original[x]['filename'])) 
                raise Exception(msg)
        original.update(new)
    
    tasks = load_configuration_entries(directory,
                                        pattern=pattern_tasks,
                                        required_fields=['id', 'desc', 'ros-node'],
                                        optional_fields=[])
    merge(Configuration.tasks, tasks)

    robots = load_configuration_entries(directory,
                                        pattern=pattern_robots,
                                        required_fields=['id', 'desc', 'ros-node'],
                                        optional_fields=[])
    merge(Configuration.robots, robots)

    agents = load_configuration_entries(directory,
                                        pattern=pattern_agents,
                                        required_fields=['id', 'desc', 'ros-node'],
                                        optional_fields=[])
    merge(Configuration.agents, agents)
        
    events = load_configuration_entries(directory,
                                        pattern=pattern_events,
                                        required_fields=['id', 'desc', 'tasks',
                                                         'agents', 'robots'],
                                        optional_fields=[])
    merge(Configuration.events, events)
    
    logger.debug('Found %5d tasks.' % len(Configuration.tasks))
    logger.debug('Found %5d robots.' % len(Configuration.robots))
    logger.debug('Found %5d agents.' % len(Configuration.agents))
    logger.debug('Found %5d events.' % len(Configuration.events))
                               
