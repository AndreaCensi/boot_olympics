
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
        self.dt = None # Time since last observations
        
        self.sensel_values = None

        self.commands = None
        self.commands_source = None 
        
        self.counter = None 
        self.id_episode = None
        self.episode_changed = None #
        
#        
#class ProblemSpec:
#
#    ''' This is the structure passed to an agent's init(). '''
#    def __init__(self):
#        self.sensels_shape = None
#        self.commands_spec = None
#        self.id_robot = None
#        self.id_actuators = None  
#        self.id_sensors = None
#        self.id_environment = None
#        self.type = None # one of "sim", "real", "log"
#
