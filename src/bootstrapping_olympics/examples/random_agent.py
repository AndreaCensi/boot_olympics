from ..interfaces import Agent

class RandomAgent(Agent):
    ''' This is the basic class for agents. '''
    
    def perform_task(self, task):
        return False
    
    
    def init_observations(self, observations_dtype, commands_spec):
        ''' 
            Provides the data-type for observations and commands,
            so that the agent can initialize its data structures.
        '''
        pass
    
    
    def process_observations(self, observations):
        '''
            Process new observations.
        '''
        pass
    
    def choose_commands(self):
        ''' Chooses commands to be generated. '''
        pass
    
    def get_state(self):
        ''' Return the state for the agent so that it can be saved. '''
        pass
    
    def set_state(self, state):
        ''' Load the given state (obtained by 'get_state'). '''
        pass
