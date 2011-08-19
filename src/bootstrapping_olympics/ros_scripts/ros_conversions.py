import numpy as np
from bootstrapping_olympics.interfaces.structures import Observations
from bootstrapping_olympics import logger

class ROS2Python():
    ''' 
        This class takes converts a BootstrappingObservations ROS structure
        into an Observation type object. It also makes some consistency
        checks, marks different episodes, discards repeated observations, etc. 
        The idea is that the data that comes out of this will not need any 
        other check by the agents. 
    '''
    
    def __init__(self, spec):
        self.last = None
        self.spec = spec
        
    def convert(self, ros_obs, filter_doubles=True):
        ''' Returns None if the same message is repeated. '''

        if filter_doubles:
            if self.last is not None and self.last.counter == ros_obs.counter:
                return None
            
        obs = Observations()
        obs.time = ros_obs.timestamp
        obs.sensel_values = np.array(ros_obs.sensel_values, dtype='float32')
        # TODO: reshape
        obs.commands = np.array(ros_obs.commands, dtype='float32')
        obs.commands_source = ros_obs.commands_source
        obs.counter = ros_obs.counter
        obs.id_episode = ros_obs.id_episode
        # obs.dt is useless
        
        current_data_description = ('[%s#%s at %s]' % 
                                    (obs.id_episode, obs.counter, obs.time))

        try:
            self.spec.check_compatible_commands_values(ros_obs.commands)
            self.spec.check_compatible_sensels_values(ros_obs.sensel_values)
        except Exception as e:
            logger.error('%s: Skipping invalid data.' % current_data_description)
            logger.error(e)
            return None
        
        if self.last is not None:
            obs.dt = obs.time - self.last.time
           
            obs.episode_changed = obs.id_episode != self.last.id_episode
            if not obs.episode_changed:
                #assert obs.dt >= 0
                if obs.dt < 0:
                    logger.error('At %s' % current_data_description)
                    logger.error('Out of order? previous time: %s current: %s dt: %s' % 
                                 (self.last.time, obs.time, obs.dt))
                    # self.last = obs
                    # self.last_ros_obs = ros_obs
                    return None
                if filter_doubles:
                    # assert obs.dt > 0
                    if obs.dt <= 0:
                        logger.error('At %s' % current_data_description)
                        logger.error('Strange, should have caught before.')
                        return None
                    
                
                if obs.dt > 0.2:
                    logger.info('Skipping %s due to strange dt %s .' % 
                                (current_data_description, obs.dt))
            else:
                obs.dt = 0.01 # episode changed
        else:
            obs.dt = 0.01
            obs.episode_changed = True
            
        self.last = obs
        self.last_ros_obs = ros_obs
        return obs
    
