from abc import abstractmethod
from collections import namedtuple

class RobotSimulationInterface:
    ''' This is the basic class for robot simulators. '''
    
    def __init__(self, observations_shape, commands_spec,
                 id_robot='unknown-robot',
                 id_sensors='unknown-sensors',
                 id_actuators='unknown-actuators'):
        # TODO: document commands_spec
        self.observations_shape = observations_shape
        self.commands_spec = commands_spec
        
        self.timestamp = 1000
        self.counter = 0
        self.id_robot = id_robot
        self.id_sensors = id_sensors
        self.id_actuators = id_actuators
        self.id_episode = "id-episode-not-set"
        self.id_environment = "id-environment-not-set"      
        self.last_commands = None
        self.last_commands_source = None
        self.last_observations = None 
          
    def next_episode(self):
        ''' 
            Changes the simulation to the next episode. 
        
            Guaranteed to be called at least once before get_observations().
            
            Should set the attributes ``id_episode`` and ``id_environment``.
        '''
        self.id_episode = 'id-episode-not-set'
        self.id_environment = 'id-episode-not-set'

    @abstractmethod
    def simulate_system(self, commands, dt):
        ''' Simulate system for dt. '''
        pass
    
    @abstractmethod
    def compute_observations(self):
        ''' Get observations. Must return a numpy array.'''
        pass
    
    #
    # Do not replace the following methods.
    #
    def get_state(self):
        ''' Return the state so that it can be saved. '''
        pass
    
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass
    
    
    # This is the data structure returned by get_last_observations()
    Observations = namedtuple('Observations',
                              'timestamp counter id_episode id_environment sensel_values '
                              'commands commands_source')
   
    def get_observations(self):
        ''' Returns an Observations structure with lots of metadata. '''
        if self.last_observations is None:
            self.compute_and_store_observations()
        return self.last_observations
        
    def apply_commands(self, commands, dt, commands_source='unknown'):
        # Simulate the system
        self.simulate_system(commands, dt)
        self.last_commands = commands
        self.last_commands_source = commands_source
        # Update counters
        self.timestamp += dt
        self.counter += 1 
        # Compute new observations 
        self.compute_and_store_observations()
        
    def compute_and_store_observations(self):
        # compute and store observations
        sensel_values = self.compute_observations()
        fields = {
          'timestamp': self.timestamp,
          'counter': self.counter,
          'id_episode': self.id_episode,
          'id_environment': self.id_environment,
          'sensel_values': sensel_values,
          'commands': self.last_commands,
          'commands_source': self.last_commands_source
        }
        # TODO: remember state
        self.last_observations = RobotSimulationInterface.Observations(**fields)
        self.counter += 1
     
