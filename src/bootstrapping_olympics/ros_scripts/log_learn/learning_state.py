from contracts import contract
import shelve
import os
from . import logger
from . import isodate
from . import expand_environment

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
         

class LearningStateDB(object):
    DEFAULT_DIR = '~/boot_learning_states/'
    
    def __init__(self, datadir):
        datadir = expand_environment(datadir)
        if not os.path.exists(datadir):
            os.makedirs(datadir)
        dbfile = os.path.join(datadir, 'agent_states.db')
        logger.debug('Loading database in %r.' % dbfile)
        self.s = shelve.open(dbfile, protocol=2, writeback=True)
        if not 'states' in self.s:
            logger.info('Created empty database.')
            self.s['states'] = {}
        
    def list_states(self):
        return self.s['states'].keys()
        
    def has_state(self, id_agent, id_robot):
        ''' Returns true if the learning state is already present. '''
        return (id_agent, id_robot) in self.s['states']
    
    def get_state(self, id_agent, id_robot):
        ''' Returns the learning state for the given combination. '''
        return self.s['states'][(id_agent, id_robot)]
    
    def set_state(self, id_agent, id_robot, state):
        ''' Sets the learning state for the given combination. '''
        self.s['states'][(id_agent, id_robot)] = state
        self.s.sync()
