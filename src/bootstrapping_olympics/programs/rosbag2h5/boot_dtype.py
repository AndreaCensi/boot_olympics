
boot_version = [1, 0, 0]

boot_description = ''' 
    This is the dtype for bootstrapping messages (version %s) 
    
    
    
    H5 files
    --------
    
    /boot_olympics_1/
    
    
''' % boot_version

boot_observations_dtype = [
    ('time', 'float64'),
                        
    ('version', ('uint8', 3)),
    ('commands_source', 'S64'),
    ('id_episode', 'S64'),
    ('id_world', 'S64'),
    ('counter', 'int'), # steps from id_episodes
        
    # redundant data
    ('dt', 'float64'),
    ('time_from_episode_start', 'float64'),
    
    
    # debug data (when created)
    ('processed-by', 'S64')
    ('processed-timestamp', 'float64') # timestamp
    ('processed-date')
    
    ('extra', 'S1000')
    # This is added by the robo
    # ('sensel_values',)     
    
]

        self.id_robot = id_robot
        self.id_episodes = id_episodes
        self.timestamp = timestamp
        self.length = length
        self.num_observations = num_observations
        self.spec = spec

        self.bag_file = bag_file
        self.topic = topic


    self.time = None
        self.dt = None # Time since last observations
        
        self.sensel_values = None

        self.commands = None
        self.commands_source = None 
        
        self.counter = None 
        self.id_episode = None
        self.episode_changed = None #
        
