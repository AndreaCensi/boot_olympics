from . import check_valid_ros_node_spec, logger
# FIXME: remove dependency
from vehicles.configuration import (wrap_check, check_generic_code_desc,
    check_necessary, load_configuration_entries)

class BootOlympicsConfig:
    tasks = {}
    robots = {}
    agents = {}
    events = {}

# TODO: deprecated
Configuration = BootOlympicsConfig

def load_boot_olympics_config(directory=None,
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
#        for x in new:
#            if x in original:
#                msg = ('Entry %r (%r) already present in %r.' % 
#                       (x, new[x]['filename'], original[x]['filename'])) 
#                raise Exception(msg)
        original.update(new)
    



    tasks = load_configuration_entries(directory,
                                        pattern=pattern_tasks,
                                        check_entry=check_valid_task_config)
    merge(Configuration.tasks, tasks)

    robots = load_configuration_entries(directory,
                                        pattern=pattern_robots,
                                        check_entry=check_valid_robot_config)
    merge(Configuration.robots, robots)

    agents = load_configuration_entries(directory,
                                        pattern=pattern_agents,
                                        check_entry=check_valid_agent_config)
    merge(Configuration.agents, agents)
        
    events = load_configuration_entries(directory,
                                        pattern=pattern_events,
                                        check_entry=check_valid_event_config)
                                        
    merge(Configuration.events, events)
    
    logger.debug('Found %5d tasks.' % len(Configuration.tasks))
    logger.debug('Found %5d robots.' % len(Configuration.robots))
    logger.debug('Found %5d agents.' % len(Configuration.agents))
    logger.debug('Found %5d events.' % len(Configuration.events))

# TODO: deprecated             
load_configuration = load_boot_olympics_config

# TODO: unit tests         
def check_valid_event_config(x):
    necessary = [('id', str),
                 ('desc', str),
                 ('tasks', list),
                 ('agents', list),
                 ('robots', list)]
    check_necessary(x, necessary)
                                                     
def check_valid_robot_config(x):
    necessary = [('id', str),
                 ('desc', str),
                 ('ros-node', list)]
    check_necessary(x, necessary)
    wrap_check(x, 'checking "code" entry',
               check_valid_ros_node_spec, x['ros-node'])


def check_valid_task_config(x):
    return check_generic_code_desc(x, 'task')

def check_valid_agent_config(x):
    return check_generic_code_desc(x, 'agent')


    
