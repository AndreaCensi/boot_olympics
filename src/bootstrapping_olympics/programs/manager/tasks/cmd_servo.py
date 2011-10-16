from . import logger, contract
from .. import check_mandatory, check_no_spurious
from bootstrapping_olympics.interfaces import AgentInterface
from bootstrapping_olympics.interfaces import ObsKeeper
from bootstrapping_olympics.interfaces import (RobotInterface,
    RobotObservations)
from bootstrapping_olympics.logs import LogsFormat
from bootstrapping_olympics.programs.manager.cmd_learn.cmd_learn import (
    load_agent_state)
from bootstrapping_olympics.utils import InAWhile
from bootstrapping_olympics.utils import isodate_with_secs
from bootstrapping_olympics.utils import natsorted
from geometry.manifolds import SE3
from geometry.poses import SE3_from_SE2, SE2_from_translation_angle
from optparse import OptionParser
import numpy as np
from geometry.yaml import to_yaml



__all__ = ['cmd_task_servo', 'task_servo']

def cmd_task_servo(data_central, argv):
    '''Simulate the interaction of an agent and a robot. ''' 
    parser = OptionParser(usage=cmd_task_servo.__doc__)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--num_episodes", type='int', default=10,
                      help="Number of episodes to simulate [%default]")
    parser.add_option("--cumulative", default=False, action='store_true',
                      help="Count already simulated episodes towards the count.")
    parser.add_option("--max_episode_len", type='float', default=30,
                      help="Maximum len of episode (seconds) [%default]")
    parser.add_option("--interval_print", type='float', default=5,
                      help='Frequency of debug messages.')
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    id_agent = options.agent
    id_robot = options.robot
    task_servo(data_central,
             id_agent=id_agent,
             id_robot=id_robot,
             max_episode_len=options.max_episode_len,
             num_episodes=options.num_episodes,
             cumulative=options.cumulative,
             interval_print=options.interval_print)

cmd_task_servo.short_usage = '''servo  -a <agent> -r <robot> '''
def task_servo(data_central, id_agent, id_robot,
               max_episode_len,
             num_episodes,
             cumulative,
                 interval_print=None, write_extra=True):
    ''' Returns the list of the episodes IDs simulated. '''
    # Instance robot object
    robot = data_central.get_bo_config().robots.instance(id_robot) #@UndefinedVariable

    boot_spec = robot.get_spec()
     
    # Instance agent object    
    
    agent, _ = load_agent_state(data_central, id_agent, id_robot,
                             reset_state=False,
                             raise_if_no_state=True)
     
    servo_agent = agent.get_servo()
    
    id_agent_servo = '%s_servo' % id_agent
          
    ds = data_central.get_dir_structure()
    id_stream = '%s-%s-%s-servo' % (id_robot, id_agent, isodate_with_secs())
    filename = ds.get_simlog_hdf_filename(id_robot=id_robot,
                                          id_agent=id_agent,
                                          id_stream=id_stream)
    logger.info('Creating stream %r\n in file %r' % (id_stream, filename))
    
    logs_format = LogsFormat.get_reader_for(filename)
    
    bk = BookkeepingServo(data_central=data_central,
                     id_robot=id_robot,
                     id_agent=id_agent_servo,
                     num_episodes=num_episodes,
                     cumulative=cumulative,
                     interval_print=interval_print)

    @contract(returns='SE2')
    def random_displacement():
        max_angle = np.deg2rad(15)
        max_t = 0.1
        t = max_t * np.random.uniform(-max_t, +max_t, 2)
        theta = np.random.uniform(-max_angle, +max_angle)
        return SE2_from_translation_angle(t, theta)
        
    if bk.another_episode_todo():
        with logs_format.write_stream(filename=filename,
                                      id_stream=id_stream,
                                      boot_spec=boot_spec) as writer:
        
            while bk.another_episode_todo():
                episode = robot.new_episode()
                # OK this only works with Vehicles
                vehicle = robot.vehicle
                obs0 = robot.get_observations().observations
                pose0 = vehicle.get_pose()
                displ = SE3_from_SE2(random_displacement())
                pose1 = SE3.multiply(pose0, displ)
                vehicle.set_pose(pose1)
                servo_agent.set_goal_observations(obs0)
                
                for observations in run_simulation_servo(id_robot, robot,
                                                   id_agent_servo, servo_agent,
                                                   100000, max_episode_len,
                                                   episode):            
                    bk.observations(observations)
                    if write_extra:
                        extra = dict(robot_state=robot.get_state(),
                                     obs0=obs0.tolist(),
                                     pose0=to_yaml('SE3', pose0),
                                     displ=to_yaml('SE3', displ),
                                     pose1=to_yaml('SE3', pose1)
                                     )
                    else:
                        extra = {}
                    writer.push_observations(observations=observations,
                                             extra=extra)
                bk.episode_done()
                
    if cumulative:
        return bk.get_all_episodes()
    else:
        return bk.get_id_episodes()
 
        
        
class BookkeepingServo():
    ''' Simple class to keep track of how many we have to simulate. '''
    def __init__(self, data_central, id_robot, id_agent, num_episodes,
                 cumulative=True, interval_print=5):
        self.data_central = data_central
        self.id_robot = id_robot
        self.cumulative = cumulative

        if self.cumulative:
            log_index = data_central.get_log_index()
            self.done_before = log_index.get_episodes_for_robot(id_robot, id_agent)
            self.num_episodes_done_before = len(self.done_before)
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



@contract(id_robot='str', id_agent='str',
          robot=RobotInterface, agent=AgentInterface, max_observations='>=1',
          max_time='>0')
def run_simulation_servo(id_robot, robot, id_agent, agent,
                         max_observations, max_time,
                         episode,
                   check_valid_values=True):
    ''' Runs an episode of the simulation. The agent should already been
        init()ed. '''
    
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
        
