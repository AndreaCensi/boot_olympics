from . import  logger, contract, np
from bootstrapping_olympics.utils import InAWhile, natsorted


class BookkeepingServo():
    ''' Simple class to keep track of how many we have to simulate. '''
    @contract(interval_print='None|>=0')
    def __init__(self, data_central, id_robot, id_agent, num_episodes,
                 cumulative=True, interval_print=5):
        self.data_central = data_central
        self.id_robot = id_robot
        self.cumulative = cumulative

        if self.cumulative:
            log_index = data_central.get_log_index()
            self.done_before = log_index.get_episodes_for_robot(id_robot,
                                                                id_agent)
            self.num_episodes_done_before = len(self.done_before)
            self.num_episodes_todo = (num_episodes -
                                      self.num_episodes_done_before)
            logger.info('Preparing to do %d episodes (already done %d).' %
                        (self.num_episodes_todo,
                         self.num_episodes_done_before))
        else:
            self.num_episodes_todo = num_episodes
            logger.info('Preparing to do %d episodes.' %
                        self.num_episodes_todo)
        self.num_episodes_done = 0
        self.num_observations = 0
        self.num_observations_episode = 0
        self.observations_per_episode = []

        self.interval_print = interval_print
        self.tracker = InAWhile(interval_print)
        self.id_episodes = set()

        try:
            from compmake import progress
            progress('Simulating episodes', (0, self.num_episodes_todo))
        except ImportError:
            pass

    def observations(self, observations):
        self.id_episodes.add(observations['id_episode'].item())

        self.num_observations_episode += 1
        self.num_observations += 1
        if self.tracker.its_time():
            msg = ('simulating %d/%d episodes obs %d (%5.1f fps)' %
                   (self.num_episodes_done,
                    self.num_episodes_todo,
                    self.num_observations, self.tracker.fps()))
            if self.num_episodes_done > 0:
                msg += (' (mean obs/ep: %.1f)' %
                        (np.mean(self.observations_per_episode)))
            logger.info(msg)

    def get_id_episodes(self):
        ''' Returns the list of episodes simulated. '''
        return natsorted(self.id_episodes)

    def get_all_episodes(self):
        ''' Returns the list of all episodes, both the already present
            and the simulated. '''
        eps = []
        eps.extend(self.id_episodes)
        eps.extend(self.done_before)
        return natsorted(set(eps))

    def episode_done(self):
        self.num_episodes_done += 1
        self.observations_per_episode.append(self.num_observations_episode)
        self.num_observations_episode = 0

        try:
            from compmake import progress
            progress('servoing', (self.num_episodes_done,
                                 self.num_episodes_todo))
        except ImportError:
            pass

    def another_episode_todo(self):
        return self.num_episodes_done < self.num_episodes_todo
