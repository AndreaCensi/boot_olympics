from . import check_no_spurious, check_mandatory, logger
from ...display import ReprepPublisher
from ...interfaces import AgentInterface
from ...utils import InAWhile, expand_environment, isodate, substitute
from ...agent_states import LearningState
from optparse import OptionParser
import numpy as np
import os
from bootstrapping_olympics.utils.scripts_utils import UserError

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
    parser.add_option("-o", dest='publish_dir', # XXX: use pattern
                      default="~/boot-learn-out/${id_agent}-${id_robot}-${date}",
                      help="Directory to store debug information [%default]")
    
    parser.add_option("--interval_save", type='int', default=300,
                      help="Interval for saving state (seconds) [%default]")
    parser.add_option("--interval_print", type='int', default=5,
                      help="Interval for printing stats (seconds) [%default]")
    
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
        
    id_agent = options.agent        
    id_robot = options.robot
        
    log_index = data_central.get_log_index()
    
    if not log_index.has_streams_for_robot(id_robot):
        msg = ('No log for robot %r found. I know: %s.' 
               % (id_robot, ", ".join(log_index.robot2streams.keys())))
        raise Exception(msg)

    bo_config = data_central.get_bo_config()

    if not id_agent in bo_config.agents:
        msg = ('Agent %r not found in configuration. I know: %s.' 
               % (options.agent, ", ".join(bo_config.agents.keys())))
        raise UserError(msg)
        
    
    AgentInterface.logger = logger # TODO: create one for agent
    
    
    agent, state = load_agent_state(data_central,
                                    id_agent=id_agent,
                                    id_robot=id_robot,
                                    reset_state=options.reset)
    
    db = data_central.get_agent_state_db()

    if True:
        from matplotlib import rc
#        rc('font', **{'family':'sans-serif', 'sans-serif':['Helvetica']})
        ## for Palatino and other serif fonts use:
        rc('font', **{'family':'serif', 'serif':['Times', 'Times New Roman',
                                                 'Palatino'],
                       'size': 9.0})
#        rc('text', usetex=True)
        

    if options.publish_interval is not None or options.once:
        pd_template = expand_environment(options.publish_dir)
        date = isodate()
        date = state.id_state
        variables = dict(id_agent=id_agent, id_robot=id_robot, date=date)
        pd = substitute(pd_template, **variables)
        logger.info('Writing output to directory %r.' % pd)
        publish_agent_output(state, agent, pd)
        
        variables['date'] = 'last'
        pd_last = substitute(pd_template, **variables)
        logger.info('Also available as %s' % pd_last)
        if os.path.exists(pd_last):
            os.unlink(pd_last)
        os.symlink(pd, pd_last)
    
    if options.once:
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

    tracker_save = InAWhile(options.interval_save)
    
    tracker = InAWhile(options.interval_print)

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
                # note: episodes not updated
                state.agent_state = agent.get_state()
                db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
    
            agent.process_observations(obs)
            
            if options.publish_interval is not None:
                if 0 == state.num_observations % options.publish_interval:
                    publish_agent_output(state, agent, pd)
                    
        state.id_episodes.update(to_learn)
        # Saving agent state
        state.agent_state = agent.get_state()
        db.set_state(state=state, id_robot=id_robot, id_agent=id_agent) 


once = False

def try_until_done(function): 
    while True:
        try:
            function()
            break
        except KeyboardInterrupt:
            logger.info('Caught CTRL-C, retrying.')
            continue   

def publish_agent_output(state, agent, pd):
    rid = ('%s-%s-%s-%07d' % (state.id_agent, state.id_robot,
                            state.id_state, state.num_observations))
    publisher = ReprepPublisher(rid)
    report = publisher.r
    
    stats = ("Num episodes: %s\nNum observations: %s" % 
             (len(state.id_episodes), state.num_observations))
    report.text('learning statistics', stats)
    
    report.text('report_date', isodate())

    agent.publish(publisher)
    filename = os.path.join(pd, '%s.html' % rid)
    global once
    if not once:
        once = True
        logger.info('Writing to %r.' % filename)
    else:
        #logger.debug('Writing to [...]/%s .' % os.path.basename(filename))
        pass
    rd = os.path.join(pd, 'images')
    report.to_html(filename, resources_dir=rd)
    
    last = os.path.join(pd, 'last.html')
    if os.path.exists(last):
        os.unlink(last)
    os.link(filename, last)
    
cmd_learn_log.short_usage = ('learn-log -a <AGENT> -r <ROBOT> '
                             ' [--reset] [--publish interval]')
    


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
        
        db.reload_state_for_agent(id_agent=id_agent, id_robot=id_robot,
                                  agent=agent)
        
    else:
        logger.info('No previous learned state found.')
        state = LearningState(id_robot=id_robot, id_agent=id_agent)

    return agent, state

