from . import StorageFilesystem, logger
from contracts import contract
import os
from bootstrapping_olympics.utils import isodate, expand_environment

__all__ = ['LearningState', 'LearningStateDB']

class LearningState(object):
    ''' 
        A learning state is the agent state + the IDs 
        of the episodes used to learn
    '''
    
    @contract(id_agent='str', id_robot='str')
    def __init__(self, id_agent, id_robot):
        self.id_agent = id_agent
        self.id_robot = id_robot
        self.id_episodes = set()
        self.num_observations = 0
        self.agent_state = None
        
        self.id_state = isodate() 
         
def key2tuple(key):
    return tuple(key.split(","))
def tuple2key(t):
    return ",".join(t)

class LearningStateDB(object):
    DEFAULT_DIR = '~/boot_learning_states/' # TODO: move in constants
    
    def __init__(self, datadir):
        datadir = expand_environment(datadir)
        if not os.path.exists(datadir):
            os.makedirs(datadir)
            
        dbdir = os.path.join(datadir, 'agent_states')
        logger.debug('Using dir %r as directory.' % dbdir)
        self.storage = StorageFilesystem(dbdir)
        
    def list_states(self):
        return [ key2tuple(x) for x in  self.storage.keys()]
        
    def has_state(self, id_agent, id_robot):
        ''' Returns true if the learning state is already present. '''
        key = tuple2key((id_agent, id_robot))
        return  key in list(self.storage.keys())
    
    def get_state(self, id_agent, id_robot):
        ''' Returns the learning state for the given combination. '''
        key = tuple2key((id_agent, id_robot))
        return self.storage.get(key)

    def set_state(self, id_agent, id_robot, state):
        ''' Sets the learning state for the given combination. '''
        key = tuple2key((id_agent, id_robot))
        return self.storage.set(key, state)
