from . import contract
import os


__all__ = ['BootStream', 'EpisodeSummary']


class EpisodeSummary:
    @contract(id_episode='str', id_agent='str', id_world='str',
              extras='list(str)', timestamp='float', length='>0',
              num_observations='int,>0')
    def __init__(self, id_episode, id_agent, id_world, num_observations,
                       length, extras, timestamp):
        self._id_episode = id_episode
        self._id_agent = id_agent
        self._id_world = id_world
        self._extras = extras
        self._timestamp = timestamp

        self._length = length
        self._num_observations = num_observations

    def get_id_episode(self):
        return self._id_episode

    def get_id_agent(self):
        return self._id_agent

    def get_timestamp(self):
        return self._timestamp

    def get_length(self):
        return self._length

    def get_num_observations(self):
        return self._num_observations

    def __str__(self):
        return ('Episode(%s,%s,%s,%ss,%s)' %
                 (self._id_episode, self._id_agent, self._id_world,
                  self._length, self._extras))


class BootStream(object):
    ''' This class represents the structure used in the .bag index. '''

    @contract(summaries='list')
    def __init__(self, id_robot, filename, topic, spec, summaries):
        self._id_robot = id_robot
        self._spec = spec
        self._episodes = summaries
        self._id_episodes = set([x.get_id_episode() for x in self._episodes])
        self._id_agents = set([x.get_id_agent() for x in self._episodes])
        self._timestamp = min([x.get_timestamp() for x in self._episodes])

        self._length = sum([x.get_length() for x in self._episodes])
        self._num_observations = sum([x.get_num_observations()
                                      for x in self._episodes])

        self._filename = filename
        self._topic = topic

        # Visualization only
        self._short_file = os.path.splitext(os.path.basename(filename))[0]

    def get_episodes(self):
        return self._episodes

    def get_id_episodes(self):
        return self._id_episodes

    def get_id_agents(self):
        return self._id_agents

    def get_id_robot(self):
        return self._id_robot

    def get_spec(self):
        return self._spec

    def get_num_observations(self):
        return self._num_observations

    def get_filename(self):
        return self._filename

    def get_topic(self):
        return self._topic

    def get_length(self):
        return self._length

    def __repr__(self):
        return 'BootStream(%s,%s,T=%s,spec=%s)' % (self._short_file,
                                         self._id_robot, self._length,
                                         self._spec)

    def read(self, only_episodes=None, read_extra=False):
        from . import LogsFormat
        reader = LogsFormat.get_reader_for(self._filename)
        # FIXME in ROS
        generator = reader.read_stream(self, only_episodes=only_episodes,
                                       read_extra=read_extra)
        for x in generator:
            yield x
