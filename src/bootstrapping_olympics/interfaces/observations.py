
class Observations:
    
    ''' This is the structure passed to an agent's process_observations(). 
    
                 time:  time as a float (seconds)
                   dt:  interval since the previous observations in the sequence.     
        sensel_values:  numpy array
             commands:  numpy array
             
        
         episode_changed: True if the episode changed, meaning that the observations
                          are not logically in sequence with previous ones.
    '''  
    def __init__(self):
        self.time = None
        
        self.sensel_values = None

        self.commands = None
        self.commands_source = None 
        
        self.dt = None # Time since last observations
        self.counter = None 
        self.id_episode = None
        self.episode_changed = None #
        
        self.id_world = None # XXX: make sure we use it
