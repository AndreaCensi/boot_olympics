from abc import ABCMeta, abstractmethod
from contracts import contract
from bootstrapping_olympics.interfaces.publisher import Publisher

__all__ = ['AgentInterface', 'UnsupportedSpec']


class AgentInterface:
    ''' 
        
    This is the interface that the agents must implement.
    
    The following is a list of conventions that we use.    
    
    Initialization
    --------------
    
    init(boot_spec) is called first. 
    process_observations() is guaranteed to be called at least once
    before choose_commands().
    
    Processing the observations
    ---------------------------
    
    The function ``process_observations()`` is given an instance
    of the class Observations, which contains many info other than
    the raw sensel values. 
    
    Logging stuff
    -------------
    
    Use the function ``self.info(msg)`` to log stuff.
    Do not use ``rospy.loginfo`` or ``print``: the agents implementation s
    hould be independent of ROS.
    
    
    Saving the state
    ----------------
    
    Implement the functions ``set_state()`` and ``get_state()``. 
    ``get_state()`` can return anything that is Pickable. 
    ``set_state()`` will be given whatever ``get_state()`` returned.
    
    
    Publishing information
    ----------------------
    
    Implement the function ``publish()`` to output information, such
    as graphs, statistics, etc. The function is given an instance of the 
    class Publisher, whose implementation is hidden.
    
    During a ROS simulation, Publisher will be a ROSPublisher that 
    will publish the data as ROS topics which can be subscribed by RViz.
    
    During offline learning, the data will be written to HTML files.

    '''

    __metaclass__ = ABCMeta

    @abstractmethod
    def init(self, boot_spec):
        ''' 
            Called when the observations and commands shape are available,
            so that the agent can initialize its data structures.
            
            The agent might throw the exception UnsupportedSpec if the 
            spec is not supported.
            
            :param boot_spec: An instance of the class BootSpec, which
             describes the specifications of the sensorimotor cascades.
                              
        '''

    @abstractmethod
    def process_observations(self, observations):
        '''
            Process new observations.
            
            :param observations: a structure of type Observations
        '''

    @abstractmethod
    @contract(returns='array')
    def choose_commands(self):
        ''' 
            Chooses commands to be generated; must return a sequence 
            of numbers or array. 
        '''

    @contract(publisher=Publisher)
    def publish(self, publisher):
        ''' 
            Publish debug information. ``publisher`` is an instance 
            of the class PublisherInterface. 
        '''

    def state_vars(self):
        return self.__dict__.keys()

    def get_state(self):
        ''' Return the state for the agent so that it can be saved. '''
        return self.get_state_vars(self.state_vars())

    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        return self.set_state_vars(state, self.state_vars())

    def info(self, msg):
        ''' Logs something. '''
        if AgentInterface.logger is not None:
            AgentInterface.logger.info(msg)

    logger = None

    def __str__(self):
        return 'Agent(%s)' % self.__class__.__name__

    def get_state_vars(self, state_vars):
        return dict((x, self.__dict__[x]) for x in state_vars)

    def set_state_vars(self, state, state_vars):
        for v in state_vars:
            if v not in state:
                self.info('Warning, no variable %r found in state vars,'
                          ' setting none.' % v)
                self.__dict__[v] = None
            else:
                if ((state[v] is None) and (v in self.__dict__) and (self.__dict__[v] is not None)):
                    print('Warning, null state %s but has value in instance' % v)
                else:
                    self.__dict__[v] = state[v]
        # self.info('State loaded: %s' % state_vars)


class ServoAgentInterface(AgentInterface):
    
    @abstractmethod
    @contract(goal='array')
    def set_goal_observations(self, goal):
        pass

class PredictorAgentInterface(AgentInterface):
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def predict_y(self, dt):
        pass

    @abstractmethod
    def estimate_u(self):
        """ Estimate current u """


class UnsupportedSpec(Exception):
    ''' Thrown by agents if they do not support the spec. '''




