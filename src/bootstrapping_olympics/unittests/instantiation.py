from bootstrapping_olympics import BootOlympicsConfig

def make_sure_loaded():
    if not BootOlympicsConfig.loaded:
        BootOlympicsConfig.load() 
        # TODO: add from environment variable


def all_robots():
    ''' Returns a list of all robots IDs. '''
    make_sure_loaded()
    return list(BootOlympicsConfig.robots.keys())


def all_agents():
    ''' Returns a list of all agents IDs. '''
    make_sure_loaded()
    return list(BootOlympicsConfig.agents.keys())

    
def get_robot(id_robot):
    return BootOlympicsConfig.robots.instance(id_robot) #@UndefinedVariable

def get_agent(id_agent):
    return BootOlympicsConfig.agents.instance(id_agent) #@UndefinedVariable
