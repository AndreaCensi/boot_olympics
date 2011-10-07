from . import check_mandatory, logger, check_no_spurious
from ... import AgentInterface, ObsKeeper, RobotObservations, RobotInterface
from ...utils import InAWhile, isodate_with_secs
from contracts import contract
from optparse import OptionParser
import tables
import time
import yaml

__all__ = ['cmd_simulate']

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
    parser.add_option("--episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--interval_print", type='float', default=5,
                      help='Frequency of debug messages.')
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    id_agent = options.agent
    id_robot = options.robot

    # Instance agent object    
    agent = data_central.get_bo_config().agents.instance(id_agent) #@UndefinedVariable
    # Instance robot object
    robot = data_central.get_bo_config().robots.instance(id_robot) #@UndefinedVariable

    boot_spec = robot.get_spec()
    
    # If --stateful is passed, we try to load a previous state.
    if options.stateful:
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
    hf = tables.openFile(filename, 'w')

    def create_table(dtype):                
        filters = tables.Filters(complevel=9, complib='zlib',
                                 fletcher32=True)
        return  hf.createTable(
                        where='/boot_olympics/logs',
                        name=id_stream,
                        description=dtype,
                        filters=filters,
                        createparents=True
                    )
    table = None
    tracker = InAWhile(options.interval_print)
    num_episodes = 0
    num_observations = 0
    for _ in range(options.num_episodes):
        num_episodes += 1
        for observations in run_simulation(id_robot, robot, agent,
                                           100000, options.episode_len):
            if table is None:
                table = create_table(observations.dtype)
            
            if tracker.its_time():
                msg = ('simulating %d/%d episodes obs %d (%5.1f fps)' % 
                       (num_episodes, options.num_episodes,
                        num_observations, tracker.fps()))
                logger.info(msg)
          

            observations = observations.reshape((1,))
            table.append(observations)

            num_observations += 1
        table.flush()
        
    hf.close() 


@contract(id_robot='str',
          robot=RobotInterface, agent=AgentInterface, max_observations='>=1',
          max_time='>0')
def run_simulation(id_robot, robot, agent, max_observations, max_time):
    episode = robot.new_episode()
    logger.debug('Episode %s' % episode)
    
    keeper = ObsKeeper(boot_spec=robot.get_spec(), id_robot=id_robot)
    keeper.new_episode_started(episode.id_episode,
                               episode.id_environment)
    counter = 0
    obs_spec = robot.get_spec().get_observations()
    cmd_spec = robot.get_spec().get_commands()
    while counter < max_observations:
        obs = robot.get_observations()
        assert isinstance(obs, RobotObservations)
        obs_spec.check_valid_value(obs.observations)
        cmd_spec.check_valid_value(obs.commands)
        
        keeper.push_data(obs.timestamp, obs.observations, obs.commands,
                         obs.commands_source)
        observations = keeper.get_observations()
        agent.process_observations(observations)
        commands = agent.choose_commands()
        robot.set_commands(commands)
        
        extra = {
                 'state': robot.get_state(),
                 'processed-timestamp': time.time()
        }
        # TODO: check dump size
        observations['extra'] = yaml.dump(extra)
        yield observations
        if observations['time_from_episode_start'] > max_time:
            break
        counter += 1
        
cmd_simulate.short_usage = ('simulate -a <AGENT> -r <ROBOT> [--num_episodes N ]'          
                            '[--episode_len N] [--save] ')
    
