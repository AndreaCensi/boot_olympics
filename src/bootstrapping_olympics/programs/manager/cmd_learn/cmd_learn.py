from . import publish_agent_output
from .. import check_no_spurious, check_mandatory, logger
from ....agent_states import LearningState
from ....interfaces import AgentInterface
from ....utils import InAWhile, UserError
from optparse import OptionParser
import numpy as np

__all__ = ['cmd_learn_log']

def cmd_learn_log(data_central, argv):
    '''Runs the learning for a given agent and log. ''' 
    parser = OptionParser(usage=cmd_learn_log.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID")
    parser.add_option("--reset", default=False, action='store_true',
                      help="Do not use cached state.")
    parser.add_option("-p", "--publish", dest='publish_interval', type='float',
                      default=None,
                      help="Publish debug information every N cycles.")
    parser.add_option("--once", default=False, action='store_true',
                      help="Just plot the published information once and exit.") 
    parser.add_option("--interval_save", type='int', default=300,
                      help="Interval for saving state (seconds) [%default]")
    parser.add_option("--interval_print", type='int', default=5,
                      help="Interval for printing stats (seconds) [%default]")
    
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    if options.publish_interval is  None and not options.once:
        msg = 'Not creating any report; pass -p <interval> or --once to do it.'
        logger.info(msg)
  
    learn_log(data_central=data_central,
              id_agent=options.agent,
              id_robot=options.robot,
              reset=options.reset,
              publish_interval=options.publish_interval,
              publish_once=options.once,
              interval_save=options.interval_save,
              interval_print=options.interval_print)
    

def learn_log(data_central, id_agent, id_robot,
              reset=False,
              publish_interval=None,
              publish_once=False,
              interval_save=None,
              interval_print=None):
    log_index = data_central.get_log_index()
    
    if not log_index.has_streams_for_robot(id_robot):
        msg = ('No log for robot %r found. I know: %s.' 
               % (id_robot, ", ".join(log_index.robots2streams.keys())))
        raise Exception(msg)

    bo_config = data_central.get_bo_config()

    if not id_agent in bo_config.agents:
        msg = ('Agent %r not found in configuration. I know: %s.' 
               % (id_agent, ", ".join(bo_config.agents.keys())))
        raise UserError(msg)
        
    
    AgentInterface.logger = logger # TODO: create one for agent
    
    
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=reset)
    
    db = data_central.get_agent_state_db()

    # TODO: move this somewhere else
    if True:
        from matplotlib import rc
#        rc('font', **{'family':'sans-serif', 'sans-serif':['Helvetica']})
        ## for Palatino and other serif fonts use:
        rc('font', **{'family':'serif', 'serif':['Times', 'Times New Roman',
                                                 'Palatino'],
                       'size': 9.0})
#        rc('text', usetex=True)


    if publish_interval is not None or publish_once:
        ds = data_central.get_dir_structure()
        report_dir = ds.get_report_dir(id_agent=id_agent,
                                       id_robot=id_robot,
                                       id_state=state.id_state)
        logger.info('Writing output to directory %r.' % report_dir)
        publish_agent_output(state, agent, report_dir)

    
    if publish_once:
        logger.info('As requested, exiting after publishing information.')
        return
    
        
    streams = log_index.get_streams_for_robot(id_robot)
    # TODO: progress bar
    num_episodes_total = 0
    num_episodes_remaining = 0
    num_observations_total = 0
    num_observations_remaining = 0
    for stream in streams:
        # Check if all learned
        num_episodes_total += len(stream.id_episodes)
        num_observations_total += stream.num_observations
        to_learn = stream.id_episodes.difference(state.id_episodes)
        if to_learn:
            num_episodes_remaining += len(to_learn)
            num_observations_remaining += stream.num_observations 
    
    template = '%20s: %7d episodes, %7d observations.'
    logger.info(template % ('total',
                            num_episodes_total,
                            num_observations_total))
    logger.info(template % ('already learned',
                            len(state.id_episodes),
                            state.num_observations)) 
    logger.info(template % ('remaining',
                            num_episodes_remaining,
                            num_observations_remaining))

    tracker_save = InAWhile(interval_save)
    
    tracker = InAWhile(interval_print)

    for stream in streams: 
        # Check if all learned
        to_learn = stream.id_episodes.difference(state.id_episodes)
        if not to_learn:
            #logger.info('Stream %s already completely learned.' % stream)
            continue
            
        cur_stream_observations = 0
        for obs in stream.read(only_episodes=to_learn): 
            state.num_observations += 1
            cur_stream_observations += 1
            
            if tracker.its_time():
                progress = 100 * (float(state.num_observations) / 
                                  num_observations_total)
                progress_log = 100 * (float(cur_stream_observations) / 
                                      stream.num_observations)
                estimated = np.NaN
                msg = ('overall %.2f%% (log %3d%%) (eps: %4d/%d, obs: %4d/%d); '
                       '%5.1f fps; remain ~%.1f minutes' % 
                       (progress, progress_log, len(state.id_episodes),
                         num_episodes_total,
                        state.num_observations, num_observations_total,
                        tracker.fps(), estimated))
                logger.info(msg)
            
            if tracker_save.its_time():
                logger.debug('Saving state (periodic)')
                # note: episodes not updated
                state.agent_state = agent.get_state()
                db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
    
            agent.process_observations(obs)
            
            if publish_interval is not None:
                if 0 == state.num_observations % publish_interval:
                    publish_agent_output(state, agent, report_dir)
                    
        state.id_episodes.update(to_learn)
        # Saving agent state
        logger.debug('Saving state (end of stream)')
        state.agent_state = agent.get_state()
        db.set_state(state=state, id_robot=id_robot, id_agent=id_agent) 


once = False

cmd_learn_log.short_usage = ('learn-log -a <AGENT> -r <ROBOT> '
                             ' [--reset] [--publish interval] [--once]')
    


def load_agent_state(data_central, id_agent, id_robot, reset_state=False):
    ''' Load the agent, loading the agent state from the state_db directory.
        If the state is not available, then it initializes anew. The
        problem spec (sensel shape, commands shape) is loaded from the 
        logs in log_directory. 
        
        Returns tuple agent, state.
    '''
    agent = data_central.get_bo_config().agents.instance(id_agent) #@UndefinedVariable

    db = data_central.get_agent_state_db()
    key = dict(id_robot=id_robot, id_agent=id_agent)
    
    index = data_central.get_log_index()
    spec = index.get_robot_spec(id_robot)
    agent.init(spec)

    if not reset_state and db.has_state(**key):
        logger.info('Using previous learned state.')
        
        state = db.reload_state_for_agent(id_agent=id_agent, id_robot=id_robot,
                                  agent=agent)
        
    else:
        logger.info('No previous learned state found.')
        state = LearningState(id_robot=id_robot, id_agent=id_agent)

    return agent, state

