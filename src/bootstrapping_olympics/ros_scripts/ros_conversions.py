import numpy as np
from bootstrapping_olympics.interfaces.structures import Observations

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
        obs.sensel_values = np.array(ros_obs.sensel_values)
        obs.commands = np.array(ros_obs.commands)
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
        else:
            obs.dt = 0.01
            obs.episode_changed = True
            
        self.last = obs
        return obs
    
