from .filesystem_storage import StorageFilesystem
from bootstrapping_olympics import logger
from bootstrapping_olympics.utils import isodate, expand_environment
from contracts import contract
import os


__all__ = ['LearningState', 'LearningStateDB']


class LearningState(object):
    ''' 
        A learning state is the agent state + the IDs 
        of the episodes used to learn
        
        # TODO: add BootSpec here
    '''

    @contract(id_agent='str', id_robot='str')
    def __init__(self, id_agent, id_robot):
        self.id_agent = id_agent
        self.id_robot = id_robot
        self.id_episodes = set()
        self.num_observations = 0
        self.agent_state = None

        self.id_state = isodate()
        
    def __str__(self):
        return ("LearningState(%s/%s;eps:%s;obs:%s;id:%s)" % 
                (self.id_agent, self.id_robot, len(self.id_episodes), self.num_observations,
                 self.id_state))

    def merge(self, other):
        assert self.id_agent == other.id_agent
        assert self.id_robot == other.id_robot
        print('merging %r and %r' % (self.id_episodes, other.id_episodes))
        self.id_episodes.update(other.id_episodes)
        self.num_observations += other.num_observations
        

def key2tuple(key):
    return tuple(key.split(","))


def tuple2key(t):
    return ",".join(t)


def isakey(t):
    return ',' in t


class LearningStateDB(object):

    def __init__(self, datadir):
        datadir = expand_environment(datadir) 

        try:  # concurrent
            if not os.path.exists(datadir):
                os.makedirs(datadir)
        except:
            pass

        self.storage = StorageFilesystem(datadir)

    def list_states(self):
        return [key2tuple(x) for x in self.storage.keys() if isakey(x)]

    def has_state(self, id_agent, id_robot):
        ''' Returns true if the learning state is already present. '''
        key = tuple2key((id_agent, id_robot))
        return key in list(self.storage.keys())

    def get_state(self, id_agent, id_robot):
        ''' Returns the learning state for the given combination. '''
        self.check_state_exists(id_agent, id_robot)
        key = tuple2key((id_agent, id_robot))
        return self.storage.get(key)

    def delete_state(self, id_agent, id_robot):
        ''' Returns the learning state for the given combination. '''
        assert self.has_state(id_agent, id_robot)
        key = tuple2key((id_agent, id_robot))
        self.storage.delete(key)

    def check_state_exists(self, id_agent, id_robot):
        if not self.has_state(id_agent, id_robot):
            msg = 'Could not find state for combination %r/%r.' % (id_agent, id_robot)
            raise ValueError(msg)

    def set_state(self, id_agent, id_robot, state):
        ''' Sets the learning state for the given combination. '''
        key = tuple2key((id_agent, id_robot))
        stats = self.storage.set(key, state)
        msg = self.storage.stats_string(stats)
        logger.debug(msg)
        return stats

    def reload_state_for_agent(self, id_agent, id_robot, agent):
        state = self.get_state(id_agent, id_robot)

        logger.debug('State after learning %d episodes.' % 
                     len(state.id_episodes))
        try:
            agent.set_state(state.agent_state)
        except:
            logger.error('Could not set agent to previous state.')
            raise
        return state

