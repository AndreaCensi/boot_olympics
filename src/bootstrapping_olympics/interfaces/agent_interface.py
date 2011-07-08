from abc import ABCMeta

# TODO: add abstract methods

class AgentInterface:
    __metaclass__ = ABCMeta
    
    ''' 
        This is the interface that the agents must implement.
    '''
    
    def perform_task(self, task):
        ''' 
            Asks the agent to perform a task. 
            To refuse (i.e., not implemented), return false.
            
            The list of tasks is found in ???.
        '''
        return False
    
    def init(self, num_sensels, commands_spec):
        ''' 
            Called when the observations and commands shape are available,
            so that the agent can initialize its data structures.
            
            :param:num_sensels: Number of sensels produced.
            :param:commands_spec: A vector with the bounds for the commands.
            
                A list of tuples (lower, upper).
                For example: [(-1,+1), (0,1)] indicates that there are two
                commands, one is bounded between -1 and +1, and the other is
                bounded between 0 and 1.
        '''
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
#    
#    def get_state(self):
#        ''' Return the state for the agent so that it can be saved. '''
#        pass
#    
#    def set_state(self, state):
#        ''' Load the given state (obtained by 'get_state'). '''
#        pass
#    
#    def info(self, msg):
#        ''' Logs something. '''
#    
#    def debug(self, msg):
#        ''' Logs something. '''
#    
#        
#    def shutdown(self):
#        pass
#    


