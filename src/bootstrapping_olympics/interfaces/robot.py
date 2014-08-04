from abc import abstractmethod
from blocks import SimpleBlackBox
from contracts import ContractsMeta, contract
from decent_logs import WithInternalLog
from streamels import BootSpec
import geometry # for geometry contracts @UnusedImport

__all__ = [
   'EpisodeDesc', 
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

    
# new_contract('RobotObservations', RobotObservations)


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
#          
    pass


class ExplorableRobot():
    
    ''' 
        This is the basic class for robots. 
        
        Protocol notes:
        
        - new_episode() must be called before get_observations()
    '''
    __metaclass__ = ContractsMeta
    
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
    @contract(returns=SimpleBlackBox)
    def get_active_stream(self):
        """ 
           
        """

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


