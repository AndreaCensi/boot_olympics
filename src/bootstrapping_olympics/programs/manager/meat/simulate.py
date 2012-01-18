from . import DirectoryStructure, logger, np, run_simulation
from .... import AgentInterface
from ....logs import LogsFormat
from ....utils import InAWhile, isodate_with_secs, natsorted
import logging


def simulate(data_central, id_agent, id_robot,
             max_episode_len,
             num_episodes,
             cumulative,
             id_episodes=None, # if None, just use the ID given by the world
             stateful=False,
             interval_print=None,
             write_extra=True):
    ''' If not cumulative, returns the list of the episodes IDs simulated,
        otherwise it returns all episodes. '''

    # Reseed the generator (otherwise multiprocessing will use the same)
    np.random.seed()

    if id_episodes is not None:
        if len(id_episodes) != num_episodes:
            raise ValueError('Expected correct number of IDs.')

    # Instance agent object    
    agent = data_central.get_bo_config().agents.instance(id_agent) #@UndefinedVariable
    # Instance robot object
    robot = data_central.get_bo_config().robots.instance(id_robot) #@UndefinedVariable

    logger = logging.getLogger("BO:%s(%s)" % (id_agent, id_robot))
    logger.setLevel(logging.DEBUG)
    AgentInterface.logger = logger # XXX

    boot_spec = robot.get_spec()

    # If --stateful is passed, we try to load a previous state.
    if stateful:
        db = data_central.get_agent_state_db()
        if db.has_state(id_agent=id_agent, id_robot=id_robot):
            logger.info('Using previous state.')
            db.reload_state_for_agent(id_agent=id_agent, id_robot=id_robot,
                                      agent=agent)
        else:
            logger.info('No previous state found.')
            agent.init(boot_spec)
    else:
        agent.init(boot_spec)

    ds = data_central.get_dir_structure()
    assert isinstance(ds, DirectoryStructure)

    id_stream = '%s-%s-%s' % (id_robot, id_agent, isodate_with_secs())
    filename = ds.get_simlog_filename(id_robot=id_robot,
                                          id_agent=id_agent,
                                          id_stream=id_stream)
    logger.info('Creating stream %r\n in file %r' % (id_stream, filename))

    logs_format = LogsFormat.get_reader_for(filename)

    bk = Bookkeeping(data_central=data_central,
                     id_robot=id_robot,
                     num_episodes=num_episodes,
                     cumulative=cumulative,
                     interval_print=interval_print)

    if bk.another_episode_todo():
        with logs_format.write_stream(filename=filename,
                                      id_stream=id_stream,
                                      boot_spec=boot_spec) as writer:

            while bk.another_episode_todo():
                if id_episodes is not None:
                    id_episode = id_episodes.pop(0)
                else:
                    id_episode = None

                for observations in run_simulation(id_robot, robot, id_agent,
                                                   agent,
                                                   100000, max_episode_len,
                                                   id_episode=id_episode):
                    bk.observations(observations)
                    if write_extra:
                        extra = dict(robot_state=robot.get_state())
                    else:
                        extra = {}
                    writer.push_observations(observations=observations,
                                             extra=extra)
                bk.episode_done()

    if cumulative:
        return bk.get_all_episodes()
    else:
        return bk.get_id_episodes()


class Bookkeeping():
    ''' Simple class to keep track of how many we have to simulate. '''
    def __init__(self, data_central, id_robot, num_episodes,
                 cumulative=True, interval_print=5):
        self.data_central = data_central
        self.id_robot = id_robot
        self.cumulative = cumulative

        if self.cumulative:
            log_index = data_central.get_log_index()
            if log_index.has_streams_for_robot(id_robot):
                self.done_before = log_index.get_episodes_for_robot(id_robot)
                self.num_episodes_done_before = len(self.done_before)
            else:
                self.done_before = set()
                self.num_episodes_done_before = 0
            self.num_episodes_todo = num_episodes - self.num_episodes_done_before
            logger.info('Preparing to do %d episodes (already done %d).' %
                        (self.num_episodes_todo, self.num_episodes_done_before))
        else:
            self.num_episodes_todo = num_episodes
            logger.info('Preparing to do %d episodes.' % self.num_episodes_todo)
        self.num_episodes_done = 0
        self.num_observations = 0
        self.num_observations_episode = 0
        self.observations_per_episode = []

        self.interval_print = interval_print
        self.tracker = InAWhile(interval_print)
        self.id_episodes = set()

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

    def another_episode_todo(self):
        return self.num_episodes_done < self.num_episodes_todo

