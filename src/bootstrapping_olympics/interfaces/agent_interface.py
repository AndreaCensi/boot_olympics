from abc import abstractmethod

class AgentInterface:
    ''' 
        This is the interface that the agents must implement.
        Note that
    '''
    
    def perform_task(self, task):
        ''' 
            Asks the agent to perform a task. 
            To refuse (i.e., not implemented), return false.
            
            The list of tasks is found in ???.
        '''
        return False
    
    def init(self):
        ''' 
            Called when the observations and commands shape are available,
            so that the agent can initialize its data structures.
        '''
        pass
    
    def shutdown(self):
        pass
    
    
    def process_observations(self, observations):
        '''
            Process new observations.
            
            :param:observations: a Numpy array containing the sensels values.
        '''
        pass
    
    def choose_commands(self):
        ''' Chooses commands to be generated; must return a sequence of numbers or array. '''
        pass
    
    def get_state(self):
        ''' Return the state for the agent so that it can be saved. '''
        pass
    
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass
    
class Agent(AgentInterface):
    
    def get_observations_shape(self):
        pass
    
    def get_commands_spec(self):
        pass
    
    def info(self, msg):
        ''' Logs something. '''
    
    def debug(self, msg):
        ''' Logs something. '''
    
        
