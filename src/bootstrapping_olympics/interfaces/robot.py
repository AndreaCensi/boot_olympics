from abc import abstractmethod
from blocks import SimpleBlackBox
from contracts import ContractsMeta, contract, new_contract
from decent_logs import WithInternalLog
from streamels import BootSpec
import geometry # for geometry contracts @UnusedImport

__all__ = [
   'EpisodeDesc', 
   'RobotObservations',
   'BasicRobot',
   'ExplorableRobot', 
   'RobotInterface',
   'PassiveRobotInterface',
]


class EpisodeDesc(object):
    ''' Structure that must be returned by new_episode(). '''
    @contract(id_episode='str', id_environment='str', extra='None|dict')
    def __init__(self, id_episode, id_environment, extra=None):
        self.id_episode = id_episode
        self.id_environment = id_environment
        self.extra = extra  # TODO: is this stored on file?

    def __str__(self):
        return "EpisodeDesc(%s,%s)" % (self.id_episode, self.id_environment)


class RobotObservations(object):
    ''' Structure that must be returned by get_observations(). '''
    
    @contract(timestamp='number',
              observations='array',
              commands='array',
              commands_source='str',
              episode_end='bool',
              robot_pose='None|array[4x4]',
              extra='dict')
    def __init__(self, timestamp, observations, commands, commands_source,
                        episode_end, robot_pose, extra={}):
        '''
            Initializes the structure.    
        
            - ``episode_end`` should indicate whether the episode ended
              due to, e.g., user intervention or collision
              (commands not accepted anymore).
            - ``robot_pose`` should be either None or a 4x4 Numpy matrix 
              representing an element of SE(3). This is used for tasks such as servoing,
              where we need to know the pose of the robot for assessing performance.
            - ``extra`` is application-specific; it will be serialized in yaml.
              
        '''
        self.timestamp = timestamp
        self.observations = observations
        self.commands = commands
        self.commands_source = commands_source
        self.episode_end = episode_end
        self.robot_pose = robot_pose
        self.extra = extra
        
    def __repr__(self):
        return 'RobotObservations(t=%s,obs=...,cmd=%s)' % (self.timestamp, self.commands)
        
    # Constant to return if the observations are not ready yet
    class NotReady(Exception):
        pass

    class Timeout(Exception):
        pass
    
    class Finished(Exception):
        pass
    
    
new_contract('RobotObservations', RobotObservations)


class BasicRobot(WithInternalLog):
    __metaclass__ = ContractsMeta
    
    @abstractmethod
    @contract(returns=BootSpec)
    def get_spec(self):
        ''' Returns the sensorimotor spec for this robot (a BootSpec object). '''
        

    @contract(returns='list[>=1]')
    def get_inner_components(self):
        """ Used internally. Used to access the inner 
            layers in case a class wraps another (like EquivRobot).
            We can get to the original robot using: :: 
                r = robot.get_inner_components()[-1]
         """ 
        return [self]
    
    
    @contract(commands='array', returns='se3')
    def debug_get_vel_from_commands(self, commands):
        """ 
            For visualization/debug purposes only: it converts a commands array
            into an element of se3 corresponding to a velocity.
        """
        
    
class PassiveRobotInterface():
    
#     @contract(returns=Source)
#     def get_passive_stream(self):
#         """ 
#             TO implement ... how to do logs?
#             
#             Returns a listening data stream for this robot as a Source block.
#             
#             The output is timestamped BootObservations instances.
#         """
        
    # @abstractmethod
    @contract(returns=RobotObservations)
    def get_observations(self):
        ''' 
            Get observations. Must return an instance of RobotObservations,
            or raise either:
            - RobotObservations.Finished => no more observations
            - RobotObservations.NotReady => not ready yet
        '''


class ExplorableRobot():
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

    @contract(returns=SimpleBlackBox)
    def get_active_stream(self):
        """ 
            TO implement ... how to do logs?
            Returns a listening data stream for this robot as a Source block.
        """
        from bootstrapping_olympics.interfaces.robot_utils import RobotAsBlackBox3

        return RobotAsBlackBox3(self)
    
    @abstractmethod
    @contract(commands='array', commands_source='str', returns='None')
    def set_commands(self, commands, commands_source):  # @UnusedVariable
        ''' Send the given commands. '''
        assert False


    # XXX: not sure
    @contract(returns='None|dict')
    def get_state(self):
        ''' 
            Return the state so that it can be saved. 
            This should be either a dictionary with YAML-serializable entries,
            or None (in which case the state will be extracted automatically).
        '''
        return None


class RobotInterface(BasicRobot, PassiveRobotInterface, ExplorableRobot):
    pass 


