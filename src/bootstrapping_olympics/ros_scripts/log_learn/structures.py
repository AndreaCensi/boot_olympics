import os

class BootStream(object):
    ''' This class represents the structure used in the .bag index. '''
    
    def __init__(self, id_robot, id_episodes, timestamp, length,
                 num_observations, bag_file, topic, sensels_shape, commands_spec):
        self.id_robot = id_robot
        self.id_episodes = id_episodes
        self.timestamp = timestamp
        self.length = length
        self.num_observations = num_observations
        self.bag_file = bag_file
        self.topic = topic
        self.short_file = os.path.splitext(os.path.basename(bag_file))[0]
        self.sensels_shape = sensels_shape
        self.commands_spec = commands_spec

    def __str__(self):
        return 'BootStream(%s,%s,T=%s,y=%s,u=%s)' % (self.short_file,
                                         self.id_robot, self.length,
                                         self.sensels_shape,
                                         self.commands_spec)

    def read(self, only_episodes=False): # TODO: implement
        from ros import rosbag #@UnresolvedImport
        bag = rosbag.Bag(self.bag_file)
        warned = False
        try:
            for msg in bag.read_messages(topics=[self.topic]):
                topic, observations, t = msg #@UnusedVariable
                if observations.id_episode == 'id-episode-not-set':
                    if not warned:
                        warned = True
                        print('Changing id_episode to %s' % self.short_file)
                    observations.id_episode = self.short_file
                # TODO: implmement only_episodes
                yield observations
        except:
            bag.close()
            raise 
