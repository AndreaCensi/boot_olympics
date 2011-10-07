from . import BootSpec
from abc import abstractmethod, ABCMeta
from contracts import  contract

class EpisodeDesc:
    ''' Structure that must be returned by new_episode(). '''
    @contract(id_episode='str', id_environment='str', extra='None|dict')
    def __init__(self, id_episode, id_environment, extra=None):
        self.id_episode = id_episode
        self.id_environment = id_environment
        self.extra = extra
        
    def __str__(self):
        return "EpisodeDesc(%s,%s)" % (self.id_episode, self.id_environment)

class RobotObservations:
    ''' Structure that must be returned by get_observations(). '''
    @contract(timestamp='number', observations='array', commands='array',
              commands_source='str')
    def __init__(self, timestamp, observations, commands, commands_source):
        self.timestamp = timestamp
        self.observations = observations
        self.commands = commands
        self.commands_source = commands_source
        
        
class RobotInterface:
    ''' This is the basic class for robot simulators. 
    
        get_spec() 
        
        new_episode() must be called before get_observations(
    
    '''
    
    __metaclass__ = ABCMeta
          
    @abstractmethod
    @contract(returns=BootSpec)
    def get_spec(self):
        ''' Returns the sensorimotor spec for this robot
            (a BootSpec object). '''
        
    @abstractmethod
    @contract(returns=EpisodeDesc)
    def new_episode(self):
        ''' 
            Skips to the next episode. 
            In real robots, the platform might return to start position.
            
            Guaranteed to be called at least once before get_observations().
            
            Should return an instance of EpisodeDesc.
        '''

    @abstractmethod
    def set_commands(self, commands):
        pass
    
    @abstractmethod
    @contract(returns=RobotObservations)
    def get_observations(self):
        ''' Get observations. Must return an instance of RobotObservations. '''
        pass
    
    @contract(returns='None|dict')
    def get_state(self):
        ''' Return the state so that it can be saved. 
            This should be either a dictionary with YAML-serializable entriee,
            or None.
        '''
        return None
    
    @contract(state='dict')
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass
    
    
#    #
#    # Do not replace the following methods.
#    #
#    # This is the data structure returned by get_last_observations()
#    # XXX: this is redundant
#    Observations = namedtuple('Observations',
#                              'timestamp counter id_episode id_environment sensel_values '
#                              'commands commands_source')
#   
#    def get_observations_wrap(self):
#        ''' Returns an Observations structure with lots of metadata. '''
#        if self.last_observations is None:
#            self.compute_and_store_observations()
#        return self.last_observations
#        
#    @contract(commands='array[K]')
#    def set_commands_wrap(self, commands, commands_source):
#        # Simulate the system
#        self.set_commands(commands)
#        self.last_commands = commands
#        self.last_commands_source = commands_source
#        self.counter += 1 
#        # Compute new observations 
#        self.compute_and_store_observations()
#        
#    def compute_and_store_observations(self):
#        # compute and store observations
#        time_sensel_tuple = self.get_observations()
#        check('tuple(float, array)', time_sensel_tuple)
#        
#        timestamp, sensel_values = time_sensel_tuple
#        
#        fields = {
#          'timestamp': timestamp,
#          'counter': self.counter,
#          'id_episode': self.id_episode,
#          'id_environment': self.id_environment,
#          'sensel_values': sensel_values,
#          'commands': self.last_commands,
#          'commands_source': self.last_commands_source
#        }
#        # TODO: remember state
#        self.last_observations = RobotInterface.Observations(**fields)
#        self.counter += 1
     