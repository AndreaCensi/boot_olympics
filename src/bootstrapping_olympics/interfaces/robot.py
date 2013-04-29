from . import contract, BootSpec
from abc import abstractmethod, ABCMeta
from contracts import new_contract

__all__ = ['EpisodeDesc', 'RobotObservations', 'RobotInterface']


class EpisodeDesc:
    ''' Structure that must be returned by new_episode(). '''
    @contract(id_episode='str', id_environment='str', extra='None|dict')
    def __init__(self, id_episode, id_environment, extra=None):
        self.id_episode = id_episode
        self.id_environment = id_environment
        self.extra = extra  # TODO: is this stored on file?

    def __str__(self):
        return "EpisodeDesc(%s,%s)" % (self.id_episode, self.id_environment)


class RobotObservations:
    ''' Structure that must be returned by get_observations(). '''
    @contract(timestamp='number',
              observations='array',
              commands='array',
              commands_source='str',
              episode_end='bool',
              robot_pose='None|array[4x4]')
    def __init__(self, timestamp, observations, commands, commands_source,
                        episode_end, robot_pose):
        '''
            Initializes the structure.    
        
            - ``episode_end`` should indicate whether the episode ended
              due to, e.g., user intervention or collision
              (commands not accepted anymore).
            - ``robot_pose`` should be either None or a 4x4 Numpy matrix 
              representing an element of SE(3). This is used for tasks such as servoing,
              where we need to know the pose of the robot for assessing performance.
        '''
        self.timestamp = timestamp
        self.observations = observations
        self.commands = commands
        self.commands_source = commands_source
        self.episode_end = episode_end
        self.robot_pose = robot_pose
        
    # Constant to return if the observations are not ready yet
    class NotReady(Exception):
        pass
    
    class Finished(Exception):
        pass
    
    
new_contract('RobotObservations', RobotObservations)


class PassiveRobotInterface(object):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    @contract(returns=BootSpec)
    def get_spec(self):
        ''' Returns the sensorimotor spec for this robot
            (a BootSpec object). '''

    @abstractmethod
    @contract(returns=RobotObservations)
    def get_observations(self):
        ''' 
            Get observations. Must return an instance of RobotObservations,
            or raise either:
            - RobotObservations.Finished => no more observations
            - RobotObservations.NotReady => not ready yet
        '''

class RobotInterface(PassiveRobotInterface):
    ''' 
        This is the basic class for robots. 
        
        Protocol notes:
        
        - new_episode() must be called before get_observations()
    '''

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
    @contract(commands='array', commands_source='str')
    def set_commands(self, commands, commands_source):
        ''' Send the given commands. '''

   
    @contract(returns='None|dict')
    def get_state(self):
        ''' 
            Return the state so that it can be saved. 
            This should be either a dictionary with YAML-serializable entries,
            or None (in which case the state will be extracted automatically).
        '''
        return None

    @contract(state='dict')
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass

