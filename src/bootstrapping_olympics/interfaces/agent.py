from abc import abstractmethod
from contracts import ContractsMeta, contract

from decent_logs import WithInternalLog
from reprep import Report
from reprep.interface import ReportInterface

__all__ = ['AgentInterface', 'UnsupportedSpec', 'ServoAgentInterface',
           'PredictorAgentInterface', 'PassiveAgentInterface']


class UnsupportedSpec(Exception):
    ''' Thrown by agents if they do not support the spec. '''


class PassiveAgentInterface(WithInternalLog):
    __metaclass__ = ContractsMeta

    class LearningConverged(Exception):
        """ 
        Thrown by process_observations() in learning agents
        to signal that they do not need more data to converge. 
        """
        pass

    
    @abstractmethod
    def process_observations(self, bd):
        '''
            Process new observations.
            
            :param bd: a numpy array with field ['observations']
           
            For learning agents, it might raise LearningConverged to signal that 
            the learning has converged and no more data is necessary. 
        '''
 
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
        
    def publish(self, pub):
        return self.display(pub)
    

    @contract(report=ReportInterface)
    def display(self, report):
        report.text('warn', 'display not implemented for %r' % type(self).__name__)

        

class ActiveAgentInterface(PassiveAgentInterface):
    
    @abstractmethod
    @contract(returns='array,finite')
    def choose_commands(self):
        ''' 
            Chooses commands to be generated; must return an 
            array that conforms to the specs.
        '''

class ServoAgentInterface(ActiveAgentInterface):
    
    @abstractmethod
    @contract(goal='array')
    def set_goal_observations(self, goal):
        pass
    
    @contract(report=Report, observations='array', goal='array')
    def display_query(self, report, observations, goal):  # @UnusedVariable
        """ Displays information regarding this particular query. """
        # report.text('warn', 'display_query not implemented for %r' % type(self).__name__)
    
        f = report.figure(cols=4)
        y0 = observations
        y_goal = goal
        
        e = y0 - y_goal
               
        # from yc1304.s10_servo_field.plots import plot_style_sensels  # XXX
        
        with f.plot('y0_vs_y_goal') as pylab:
            pylab.plot(y0, '.', label='y0')
            pylab.plot(y_goal, '.', label='ygoal')
            # plot_style_sensels(pylab)   
            pylab.legend()
         
        with f.plot('error') as pylab:
            pylab.plot(e, label='error')
     
    def get_distance(self, y1, y2):
        import numpy as np
        return np.linalg.norm(y1 - y2)
    
class PredictorAgentInterface(PassiveAgentInterface):
    
    @abstractmethod
    @contract(returns='array')
    def predict_y(self, dt):
        pass

    @abstractmethod
    @contract(returns='array')
    def estimate_u(self):
        """ Estimate current u """


    

class AgentInterface(PassiveAgentInterface):
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

    @contract(returns=PredictorAgentInterface)
    def get_predictor(self):
        raise NotImplementedError()

    @contract(returns=ServoAgentInterface)
    def get_servo(self):
        raise NotImplementedError()

    @contract(i='int,>=0,i', n='int,>=1,>=i')
    def parallel_process_hint(self, i, n):
        """ 
            Hint for parallel processing. It tells this instance that
            it is instance "i" of "n" that sees the same data.
            
            Learning modality: 
            1) N copies of the same thing that looks at the same data
               Then parallel_process_hint(i, N) is called for each one.
               
            2) Different learners look at the same thing.
                Then parallel_process_hint(0, 1) is called for all learners.
        """
        raise NotImplementedError()
    
    def merge(self, other):
        raise NotImplementedError()
    
    
    
    
    # Serialization stuff
    
    def state_vars(self):
        # print('default state vars for %s' % type(self))
        return self.__dict__.keys()

    def get_state(self):
        ''' Return the state for the agent so that it can be saved. '''
        return self.get_state_vars(self.state_vars())

    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        return self.set_state_vars(state, self.state_vars())

    def __str__(self):
        return 'Agent(%s)' % self.__class__.__name__

    def get_state_vars(self, state_vars):
        return dict((x, self.__dict__[x]) for x in state_vars)

    def set_state_vars(self, state, state_vars):
        # print('Setting state vars: %s' % state_vars)
        for v in state_vars:
            if v not in state:
                self.info('Warning, no variable %r found in state vars,'
                          ' setting none.' % v)
                self.__dict__[v] = None
            else:
                if ((state[v] is None) and (v in self.__dict__) and 
                    (self.__dict__[v] is not None)):
                    print('Warning, null state %s but has value in instance' % v)
                else:
                    self.__dict__[v] = state[v]
        # self.info('State loaded: %s' % state_vars)
