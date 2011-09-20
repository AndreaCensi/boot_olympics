from abc import abstractmethod, ABCMeta
from collections import namedtuple
from contracts import check, contract

class RobotInterface:
    ''' This is the basic class for robot simulators. '''
    
    __metaclass__ = ABCMeta
    
    def __init__(self, observations_shape, commands_spec,
                 id_robot='unknown-robot',
                 id_sensors='unknown-sensors',
                 id_actuators='unknown-actuators'):
        # TODO: document commands_spec
        self.observations_shape = observations_shape
        self.commands_spec = commands_spec
        
        self.counter = 0
        self.id_robot = id_robot
        self.id_sensors = id_sensors
        self.id_actuators = id_actuators
        self.id_episode = "id-episode-not-set"
        self.id_environment = 'id_environment_not_set'      
        self.last_commands = None
        self.last_commands_source = None
        self.last_observations = None 
          
    @abstractmethod
    def new_episode(self):
        ''' 
            Changes the simulation to the next episode. 
        
            Guaranteed to be called at least once before get_observations().
            
            Should set the attributes ``id_episode`` and ``id_environment``.
        '''
        self.id_episode = 'id-episode-not-set'
        self.id_environment = 'id_environment_not_set'

    @abstractmethod
    def set_commands(self, commands):
        pass
    
    @abstractmethod
    def get_observations(self):
        ''' Get observations. Must return a tuple of two elements:
            ``(timestamp, sensels)`` respectively a float and a numpy array. '''
        pass
    
    def get_state(self):
        ''' Return the state so that it can be saved. '''
        pass
    
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass
    
    
    #
    # Do not replace the following methods.
    #
    # This is the data structure returned by get_last_observations()
    Observations = namedtuple('Observations',
                              'timestamp counter id_episode id_environment sensel_values '
                              'commands commands_source')
   
    def get_observations_wrap(self):
        ''' Returns an Observations structure with lots of metadata. '''
        if self.last_observations is None:
            self.compute_and_store_observations()
        return self.last_observations
        
    @contract(commands='array[K]')
    def set_commands_wrap(self, commands, commands_source):
        # Simulate the system
        self.set_commands(commands)
        self.last_commands = commands
        self.last_commands_source = commands_source
        self.counter += 1 
        # Compute new observations 
        self.compute_and_store_observations()
        
    def compute_and_store_observations(self):
        # compute and store observations
        time_sensel_tuple = self.get_observations()
        check('tuple(float, array)', time_sensel_tuple)
        
        timestamp, sensel_values = time_sensel_tuple
        
        fields = {
          'timestamp': timestamp,
          'counter': self.counter,
          'id_episode': self.id_episode,
          'id_environment': self.id_environment,
          'sensel_values': sensel_values,
          'commands': self.last_commands,
          'commands_source': self.last_commands_source
        }
        # TODO: remember state
        self.last_observations = RobotInterface.Observations(**fields)
        self.counter += 1
     
