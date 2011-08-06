import numpy as np
from bootstrapping_olympics.interfaces.structures import Observations
from bootstrapping_olympics import logger

class ROS2Python():
    ''' 
        This class takes converts a BootstrappingObservations ROS structure
        into an Observation type object. It also makes some consistency
        checks, marks different episodes, etc. The idea is that the
        data that comes out of this will not need any other checks 
        by the agents. 
    '''
    
    def __init__(self):
        self.last = None
        
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
        
        
        current_data_description = ('[%s#%s at %s (dt: %s)]' % 
                                    (obs.id_episode, obs.counter, obs.time, obs.dt))
                
        if self.last is not None:
            # check that some things are conserved
            sensel_shape = obs.sensel_values.shape
            sensel_shape_required = self.last.sensel_values.shape
            if sensel_shape != sensel_shape_required:
                logger.info('Skipping %s because observations shape is %s instead of %s.' % 
                            (current_data_description, sensel_shape, sensel_shape_required))
                return None

            commands_shape = obs.commands.shape
            commands_shape_required = self.last.commands.shape
            if commands_shape != commands_shape_required:
                logger.info('Skipping %s because commands shape is %s instead of %s.' % 
                            (current_data_description, commands_shape, commands_shape_required))
                return None
            
            obs.dt = obs.time - self.last.time
            obs.episode_changed = obs.id_episode != self.last.id_episode
            if not obs.episode_changed:
                assert obs.dt >= 0
                if filter_doubles:
                    assert obs.dt > 0
                     
                
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
    
