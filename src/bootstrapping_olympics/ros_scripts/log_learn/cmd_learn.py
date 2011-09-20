from . import logger
from ...configuration import BootOlympicsConfig
from ...display import ReprepPublisher
from ...interfaces import AgentInterface
from ...utils import InAWhile, expand_environment, isodate
from ..agent_states import LearningState, LearningStateDB
from ..logs import LogIndex
from conf_tools import instantiate_spec
from optparse import OptionParser
from pprint import pformat
from string import Template
import numpy as np
import os

__all__ = ['cmd_learn_log']

def cmd_learn_log(main_options, argv):
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
    
    if args:
        raise Exception('Spurious arguments: %s' % args)
    
    if options.agent is None:
        msg = 'Please provide agent ID with --agent.'
        raise Exception(msg)
        
    
    id_agent = options.agent

    if options.robot is None:
        msg = 'Please provide robot ID with --robot.'
        raise Exception(msg)
        
    id_robot = options.robot
        
    index = LogIndex()
    index.index(main_options.log_directory)
    
    if not id_robot in index.robot2streams:
        msg = ('No log for robot %r found. I know: %s.' 
               % (id_robot, ", ".join(index.robot2streams.keys())))
        raise Exception(msg)
        

    if not id_agent in BootOlympicsConfig.agents:
        msg = ('Agent %r not found in configuration. I know: %s.' 
               % (options.agent, ", ".join(BootOlympicsConfig.agents.keys())))
        raise Exception(msg)
        
    
    AgentInterface.logger = logger # TODO: create one for agent
    
    agent_spec = BootOlympicsConfig.agents[id_agent]
       
    
    agent, state = load_agent_and_state(
                    agent_spec=agent_spec,
                    id_agent=id_agent,
                    id_robot=id_robot,
                    state_db_directory=main_options.state_directory,
                    log_directory=main_options.log_directory,
                    reset_state=options.reset)

    db = LearningStateDB(main_options.state_directory)

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
        pd = substitute(pd_template, variables)
        logger.info('Writing output to directory %r.' % pd)
        publish_agent_output(state, agent, pd)
        
        variables['date'] = 'last'
        pd_last = substitute(pd_template, variables)
        logger.info('Also available as %s' % pd_last)
        if os.path.exists(pd_last):
            os.unlink(pd_last)
        os.symlink(pd, pd_last)
    
    if options.once:
        logger.info('As requested, exiting after publishing information.')
        return 0
    
        
    streams = index.robot2streams[id_robot]
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
    

def substitute(template, variables):
    ''' Wrapper around Template.substitute for better error display. '''
    try:
        return Template(template).substitute(variables)
    except KeyError as e:
        msg = ('Error while substituting in string %r. Key %s not found: '
               'available keys are %s.' % (template, e, variables.keys()))
        raise Exception(msg)
    

def load_agent_and_state(agent_spec, id_agent, id_robot,
                         state_db_directory, log_directory, reset_state=False):
    ''' Load the agent, loading the agent state from the state_db directory.
        If the state is not available, then it initializes anew. The
        problem spec (sensel shape, commands shape) is loaded from the 
        logs in log_directory. 
        
        Returns tuple agent, state.
    '''
    logger.info('Instancing agent spec:\n%s' % pformat(agent_spec))
    agent = instantiate_spec(agent_spec['code']) # XXX    
    db = LearningStateDB(state_db_directory)
    key = dict(id_robot=id_robot, id_agent=id_agent)
    
    # XXX: I'm not sure this is the right order
    # XXX: not sure the smartest thing to do:
    
    index = LogIndex()
    index.index(log_directory)
    spec = index.robot2streams[id_robot][0].spec
    sensel_shape = spec.sensels_shape
    commands_spec = spec.commands_spec
    logger.info('Agent init Sensels: %s  commands: %s' % 
                (sensel_shape, commands_spec))
    agent.init(sensel_shape, commands_spec) # XXX: put new initialization

    if not reset_state and db.has_state(**key):
        logger.info('Using previous learned state.')
        state = db.get_state(**key)
        logger.info('State after learning %d episodes.' % 
                    len(state.id_episodes))
        try:
            agent.set_state(state.agent_state)
        except:
            logger.error('Could not set agent to previous state.')
            raise 
    else:
        logger.info('No previous learned state found.')
        state = LearningState(id_robot=id_robot, id_agent=id_agent)

    return agent, state

