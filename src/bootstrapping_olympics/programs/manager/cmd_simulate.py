from . import (check_mandatory, logger, check_no_spurious, contract, np,
    OptionParser)
from ... import AgentInterface, ObsKeeper, RobotObservations, RobotInterface
from ...logs import LogsFormat
from ...utils import InAWhile, isodate_with_secs, natsorted
import logging


__all__ = ['cmd_simulate', 'simulate']

def cmd_simulate(data_central, argv):
    '''Simulate the interaction of an agent and a robot. ''' 
    parser = OptionParser(usage=cmd_simulate.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--stateful", default=False, action='store_true',
                      help="Save/load the state of the agent.")
    parser.add_option("--num_episodes", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--cumulative", default=False, action='store_true',
                      help="Count already simulated episodes towards the count.")
    parser.add_option("--episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--interval_print", type='float', default=5,
                      help='Frequency of debug messages.')
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    id_agent = options.agent
    id_robot = options.robot
    simulate(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             max_episode_len=options.episode_len,
             num_episodes=options.num_episodes,
             stateful=options.stateful,
             interval_print=options.interval_print,
             cumulative=options.cumulative)
    
def simulate(data_central, id_agent, id_robot,
             max_episode_len,
             num_episodes,
             cumulative,
             stateful=False,
             interval_print=None,
             write_extra=True):
    ''' If not cumulative, returns the list of the episodes IDs simulated,
        otherwise it returns all episodes. ''' 
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
    id_stream = '%s-%s-%s' % (id_robot, id_agent, isodate_with_secs())
    filename = ds.get_simlog_hdf_filename(id_robot=id_robot,
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
                for observations in run_simulation(id_robot, robot, id_agent, agent,
                                                   100000, max_episode_len):            
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

cmd_simulate.short_usage = ('simulate -a <AGENT> -r <ROBOT> [--num_episodes N ]'          
                            '[--episode_len N] [--save] ')
    


@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, agent=AgentInterface, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, id_agent, agent, max_observations, max_time,
                   check_valid_values=True):
    ''' Runs an episode of the simulation. The agent should already been
        init()ed. '''
    episode = robot.new_episode()
    logger.debug('Episode %s' % episode)

    keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)
    keeper.new_episode_started(episode.id_episode,
                               episode.id_environment)
    counter = 0
    obs_spec = robot.get_spec().get_observations()
    cmd_spec = robot.get_spec().get_commands()
    
    def get_observations():
        obs = robot.get_observations()
        if check_valid_values:
            assert isinstance(obs, RobotObservations)
            obs_spec.check_valid_value(obs.observations)
            cmd_spec.check_valid_value(obs.commands)
        
        keeper.push_data(obs.timestamp, obs.observations, obs.commands,
                         obs.commands_source)
        observations = keeper.get_observations()
        
        if check_valid_values:
            obs_spec.check_valid_value(observations['observations'])
            cmd_spec.check_valid_value(observations['commands'])
        return observations
    
    commands = agent.choose_commands() # repeated
    while counter < max_observations:
       
        if check_valid_values:
            cmd_spec.check_valid_value(commands)
        
        robot.set_commands(commands, id_agent)
        observations = get_observations()
        
        yield observations
        
        if observations['time_from_episode_start'] > max_time:
            break
        
        if robot.episode_ended(): # Fishy
            break
    
        agent.process_observations(observations)
        commands = agent.choose_commands() # repeated
            
        counter += 1
        
