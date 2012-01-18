from . import contract, np


''' 
    This is the structure passed to an agent's process_observations(). 

    It contains many fields that lift some of the boring work from
    writing agents. 
    
             time:  time as a float (seconds)
               dt:  interval since the previous observations in the sequence.     
    sensel_values:  numpy array
         commands:  numpy array
         
    
     episode_start: True if the episode changed, meaning that the observations
                      are not logically in sequence with previous ones.
'''

boot_observations_version = [1, 0]

boot_observations_dtype = [
    ('timestamp', 'float64'),
    # ('observations',) # This will be added when the shape is known. 
    # ('commands',)  # This will be added later.

    ('version', ('uint8', 2)),
    ('commands_source', 'S64'),
    ('id_robot', 'S64'),
    ('id_episode', 'S64'),
    ('id_world', 'S64'),
    ('counter', 'int'), # steps from id_episodes
    ('episode_start', 'bool'), # True if episode started

    # redundant data
    ('dt', 'float64'), # time from last observations. This is 0 if counter = 0
    ('time_from_episode_start', 'float64'),


    ('extra', 'object')
    # extra['robot_state'] # robot state
]


def get_observations_dtype(boot_spec):
    dtype = list(boot_observations_dtype)
    dtype.append(('observations', 'float32',
                  boot_spec.get_observations().shape()))
    dtype.append(('commands', 'float32',
                  boot_spec.get_commands().shape()))
    return np.dtype(dtype)


class ObsKeeper: # TODO: move away from here
    ''' This is a simple utility class to fill in the redundant 
        fields in the Observations class. '''

    def __init__(self, boot_spec, id_robot):
        self.episode_started = False
        self.observations = None
        self.boot_spec = boot_spec
        self.dtype = get_observations_dtype(boot_spec)
        self.id_robot = id_robot
        self.id_episode = None
#        
#    def new_episode_started(self, id_episode, id_world):
#        self.id_episode = id_episode
#        self.id_world = id_world
#        self.counter = 0
#        self.timestamp_start = None
#        self.episode_started = True
#        self.observations = None
#        self.last_observations = None
#                

    @contract(timestamp='number', observations='array', commands='array',
              commands_source='str')
    def push(self, timestamp, observations, commands, commands_source,
                   id_episode, id_world):
        ''' Returns the observations structure. '''

        if self.id_episode != id_episode:
            self.id_episode = id_episode
            self.id_world = id_world
            self.counter = 0
            self.timestamp_start = timestamp
            self.episode_started = True
            self.observations = None
            self.last_observations = None

        # TODO: check timestamp order 

#        if not self.episode_started:
#            msg = 'Episode not started yet.' 
#            raise ValueError(msg)
#        if  self.timestamp_start is  None:
#            self.timestamp_start = timestamp

        # TODO: add option here
        self.boot_spec.get_observations().check_valid_value(observations)
        self.boot_spec.get_commands().check_valid_value(commands)

        x = np.zeros((), self.dtype)
        x['counter'] = self.counter
        x['id_robot'] = self.id_robot
        x['timestamp'] = timestamp
        x['observations'][:] = observations[:]
        x['commands'][:] = commands[:]
        x['commands_source'] = commands_source
        x['time_from_episode_start'] = timestamp - self.timestamp_start
        x['id_episode'] = self.id_episode
        x['id_world'] = self.id_world

        if self.last_observations is None:
            x['dt'] = 0
            x['episode_start'] = True

        else:
            x['dt'] = x['timestamp'] - self.last_observations['timestamp']
            x['episode_start'] = False

        x['extra'] = None

        self.last_observations = x
        self.counter += 1

        return x
#        
#    def get_observations(self):
#        if self.observations is None:
#            msg = 'No data pushed yet.'
#            raise ValueError(msg)
#        return self.observations
#        
