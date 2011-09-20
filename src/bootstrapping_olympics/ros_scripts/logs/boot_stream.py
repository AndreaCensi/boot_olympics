import os
from bootstrapping_olympics.ros_scripts.logs.logs_format import LogsFormat

__all__ = ['BootStream']

class BootStream(object):
    ''' This class represents the structure used in the .bag index. '''
    
    def __init__(self, id_robot, id_episodes, timestamp, length,
                 num_observations, bag_file, topic, spec):
        self.id_robot = id_robot
        self.id_episodes = id_episodes
        self.timestamp = timestamp
        self.length = length
        self.num_observations = num_observations
        self.bag_file = bag_file
        self.topic = topic
        self.spec = spec

        # Visualization only
        self.short_file = os.path.splitext(os.path.basename(bag_file))[0]

    def __str__(self):
        return 'BootStream(%s,%s,T=%s,spec=%s)' % (self.short_file,
                                         self.id_robot, self.length,
                                         self.spec)

    def read(self, only_episodes=False): # TODO: implement
        reader = LogsFormat.get_reader_for(self.bag_file)
        generator = reader.read_stream(self) 
        for x in generator:
            yield x
