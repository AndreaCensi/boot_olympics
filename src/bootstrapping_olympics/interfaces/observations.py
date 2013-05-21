from .boot_spec import BootSpec
from .streamels import BOOT_OLYMPICS_SENSEL_RESOLUTION
from contracts import contract
import numpy as np

__all__ = ['boot_observations_dtype', 'get_observations_dtype', 'ObsKeeper']

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

# TODO: check this somewhere
max_identifiers_size = 256
strings_format = 'S%d' % max_identifiers_size

boot_observations_dtype = [
    ('timestamp', 'float64'),
    # ('observations',) # This will be added when the shape is known. 
    # ('commands',)  # This will be added later.

    ('version', ('uint8', 2)),
    ('commands_source', strings_format),
    ('id_robot', strings_format),
    ('id_episode', strings_format),
    ('id_world', strings_format),
    ('counter', 'int'),  # steps from id_episodes
    ('episode_start', 'bool'),  # True if episode started

    # redundant data
    ('dt', 'float64'),  # time from last observations. This is 0 if counter = 0
    ('time_from_episode_start', 'float64'),


    ('extra', 'object')
    # extra['robot_state'] # robot state
]


@contract(boot_spec=BootSpec)
def get_observations_dtype(boot_spec):
    dtype = list(boot_observations_dtype)
    # ok, here we go. 
    dtype.append(('observations', BOOT_OLYMPICS_SENSEL_RESOLUTION,
                  boot_spec.get_observations().shape()))
    dtype.append(('commands', BOOT_OLYMPICS_SENSEL_RESOLUTION,
                  boot_spec.get_commands().shape()))
    return np.dtype(dtype)


class ObsKeeper(object):  # TODO: move away from here
    ''' This is a simple utility class to fill in the redundant 
        fields in the Observations class. '''

    def __init__(self, boot_spec, id_robot, check_valid_values=True):
        '''
            Initializes the object.
            
            :param check_valid_values: checks that the commands and 
                observations respect the spec
        '''
        if len(id_robot) > max_identifiers_size:
            msg = "Robot id %r is too long." % id_robot
            raise ValueError(msg)
        self.episode_started = False
        self.observations = None
        self.boot_spec = boot_spec
        self.dtype = get_observations_dtype(boot_spec)
        self.id_robot = id_robot
        self.id_episode = None
        self.check_valid_values = check_valid_values

    @contract(timestamp='number',
              observations='array',
              commands='array',
              commands_source='str',
              id_episode='str',
              id_world='str',
              extra='dict'
              )
    def push(self, timestamp, observations, commands, commands_source,
                   id_episode, id_world, extra={}):
        ''' Returns the observations structure. '''

        if len(id_episode) > max_identifiers_size:
            msg = "Episode id %r is too long." % id_episode
            raise ValueError(msg)

        if len(id_world) > max_identifiers_size:
            msg = "World id %r is too long." % id_world
            raise ValueError(msg)

        if self.id_episode != id_episode:
            self.id_episode = id_episode
            self.id_world = id_world
            self.counter = 0
            self.timestamp_start = timestamp
            self.episode_started = True
            self.observations = None
            self.last_observations = None

        # TODO: check timestamp order  

        # TODO: add option here
        if self.check_valid_values:
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

        x['extra'] = extra

        self.last_observations = x
        self.counter += 1

        return x

