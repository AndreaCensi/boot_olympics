import numpy as np
from bootstrapping_olympics.interfaces.structures import Observations
from bootstrapping_olympics import logger

class ROS2Python():
    
    def __init__(self):
        self.last = None
        
    def convert(self, ros_obs, filter_doubles=True):
        ''' Returns None if the same message is repeated. '''
#        print('found: %s %s %s' % 
#              (ros_obs.id_episode, ros_obs.counter, ros_obs.timestamp))
#
        if filter_doubles:
            if self.last is not None and self.last.counter == ros_obs.counter:
                #print('Same observations: %s %s' % 
                #      (ros_obs.id_episode, ros_obs.counter))
                return None
        
        obs = Observations()
        obs.time = ros_obs.timestamp
        obs.sensel_values = np.array(ros_obs.sensel_values, dtype='float32')
        obs.commands = np.array(ros_obs.commands, dtype='float32')
        obs.commands_source = ros_obs.commands_source
        obs.counter = ros_obs.counter
        obs.id_episode = ros_obs.id_episode
        
        
        if self.last is not None:
            obs.dt = obs.time - self.last.time
            obs.episode_changed = obs.id_episode != self.last.id_episode
            if not obs.episode_changed:
                assert obs.dt >= 0
                if filter_doubles:
                    assert obs.dt > 0
                    
                if obs.dt > 0.2:
                    logger.info('Skipping episode %s counter %s time %s dt %s due to strange dt.' % 
                                (obs.id_episode, obs.counter, obs.time, obs.dt))
            else:
                obs.dt = 0.01 # episode changed
        else:
            obs.dt = 0.01
            obs.episode_changed = True
            
        self.last = obs
        return obs
    
