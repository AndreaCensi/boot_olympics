import os
from . import contract

__all__ = ['BootStream']

class EpisodeSummary:
    @contract(id_episode='str', id_agent='str', id_world='str',
              extras='list(str)')
    def __init__(self, id_episode, id_agent, id_world, num_observations,
                       length, extras):
        self.id_episode = id_episode
        self.id_agent = id_agent
        self.id_world = id_world
        self.extras = extras
        self.length = length
        
    def __str__(self):
        return ('Episode(%s,%s,%s,%ss,%s)' % 
                 (self.id_episode, self.id_agent, self.id_world, self.length,
                  self.extras))
        
class BootStream(object):
    ''' This class represents the structure used in the .bag index. '''
    
    @contract(summaries='list')
    def __init__(self, id_robot, id_episodes, timestamp, length,
                 num_observations, bag_file, topic, spec, id_agents,
                 summaries):
        self.id_robot = id_robot
        self.id_episodes = id_episodes
        self.timestamp = timestamp
        self.length = length
        self.num_observations = num_observations
        self.spec = spec
        self.id_agents = set(id_agents)
        
        self.episodes = summaries 

        # TODO: change with id_stream, filename
        self.bag_file = bag_file
        self.topic = topic

        # Visualization only
        self.short_file = os.path.splitext(os.path.basename(bag_file))[0]

        
    def __repr__(self):
        return 'BootStream(%s,%s,T=%s,spec=%s)' % (self.short_file,
                                         self.id_robot, self.length,
                                         self.spec)

    def read(self, only_episodes=False, read_extra=False): # TODO: implement
        from . import LogsFormat
        reader = LogsFormat.get_reader_for(self.bag_file)
        generator = reader.read_stream(self, read_extra=read_extra) 
        for x in generator:
            yield x
