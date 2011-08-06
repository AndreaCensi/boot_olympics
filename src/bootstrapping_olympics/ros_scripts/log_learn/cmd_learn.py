from . import (LearningState, LearningStateDB, bag_get_index_object, logger,
    expand_environment, isodate)
from ...loading import BootOlympicsConfig, instantiate_spec
from bootstrapping_olympics.ros_scripts.log_learn.reprep_publisher import (
    ReprepPublisher)
from optparse import OptionParser
from pprint import pformat
from string import Template
import numpy as np
import os
from . import InAWhile
from bootstrapping_olympics.ros_scripts.ros_conversions import ROS2Python
from bootstrapping_olympics.interfaces import AgentInterface


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
    parser.add_option("-o", dest='publish_dir',
                      default="~/boot-learn-out/${id_agent}-${id_robot}-${date}",
                      help="Directory to store debug information [%default]")
    
    (options, args) = parser.parse_args(argv)
    
    if args:
        logger.error('Spurious arguments: %s' % args)
        return -1
    
    if options.agent is None:
        msg = 'Please provide agent ID with --agent.'
        logger.error(msg)
        return -1
    
    id_agent = options.agent

    if options.robot is None:
        msg = 'Please provide robot ID with --robot.'
        logger.error(msg)
        return -1
    id_robot = options.robot
    
    
    index = bag_get_index_object(main_options.log_directory)
    robots = index['robots']
    
    if not id_robot in robots:
        msg = ('No log for robot %r found. I know: %s.' 
               % (id_robot, ", ".join(robots.keys())))
        logger.error(msg)
        return -3

    if not id_agent in BootOlympicsConfig.agents:
        msg = ('Agent %r not found in configuration. I know: %s.' 
               % (options.agent, ", ".join(BootOlympicsConfig.agents.keys())))
        logger.error(msg)
        return -3
    
    AgentInterface.logger = logger # TODO: create one for agent
    
    agent_spec = BootOlympicsConfig.agents[id_agent]
       
    
    agent, state = load_agent_and_state(
                    agent_spec=agent_spec, id_agent=id_agent, id_robot=id_robot,
                       state_db_directory=main_options.state_directory,
                       log_directory=main_options.log_directory,
                       reset_state=options.reset)

    db = LearningStateDB(main_options.state_directory)

    if options.publish_interval is not None:
        pd_template = expand_environment(options.publish_dir)
        date = isodate()
        date = state.id_state
        vars = dict(id_agent=id_agent, id_robot=id_robot, date=date)
        pd = substitute(pd_template, vars)
        logger.info('Writing output to directory %r.' % pd)
        publish_agent_output(state, agent, pd)
        
        vars['date'] = 'last'
        pd_last = substitute(pd_template, vars)
        logger.info('Also available as %s' % pd_last)
        if os.path.exists(pd_last):
            os.unlink(pd_last)
        os.symlink(pd, pd_last)
    
    if options.once:
        logger.info('As requested, exiting after publishing information.')
        return 0
    
        
    # TODO: progress bar
    num_episodes_total = 0
    num_episodes_remaining = 0
    num_observations_total = 0
    num_observations_remaining = 0
    for stream in robots[id_robot]:
        # Check if all learned
        num_episodes_total += len(stream.id_episodes)
        num_observations_total += stream.num_observations
        to_learn = stream.id_episodes.difference(state.id_episodes)
        if to_learn:
            num_episodes_remaining += len(to_learn)
            num_observations_remaining += stream.num_observations 
    
    template = '%20s: %7d episodes, %7d observations.'
    logger.info(template % ('total', num_episodes_total, num_observations_total))
    logger.info(template % ('already learned',
                            len(state.id_episodes),
                            state.num_observations)) 
    logger.info(template % ('remaining',
                            num_episodes_remaining,
                            num_observations_remaining))



    tracker = InAWhile(5)
    ros2python = ROS2Python()
    for stream in robots[id_robot]:
        # Check if all learned
        to_learn = stream.id_episodes.difference(state.id_episodes)
        if not to_learn:
            #logger.info('Stream %s already completely learned.' % stream)
            continue
            
        cur_stream_observations = 0
        for ros_obs in stream.read(only_episodes=to_learn):
            obs = ros2python.convert(ros_obs)
            if obs is None: # repeated message
                continue
            
            state.num_observations += 1
            cur_stream_observations += 1
            
            if tracker.its_time():
                progress = 100 * float(state.num_observations) / num_observations_total
                progress_log = 100 * float(cur_stream_observations) / stream.num_observations
                estimated = np.NaN
                msg = ('overall %.2f%% (log %3d%%) (eps: %4d/%d, obs: %4d/%d); '
                       '%5.1f fps; remain ~%.1f minutes' % 
                       (progress, progress_log, len(state.id_episodes), num_episodes_total,
                        state.num_observations, num_observations_total,
                        tracker.fps(), estimated))
                logger.info(msg)
            
            agent.process_observations(obs)
            
            if options.publish_interval is not None:
                if 0 == state.num_observations % options.publish_interval:
                    publish_agent_output(state, agent, pd)
                    
        state.id_episodes.update(to_learn)
        # Saving agent state
        state.agent_state = agent.get_state()
        
        def save_state(): 
            db.set_state(state=state, id_robot=id_robot, id_agent=id_agent)
        try_until_done(save_state)

    logger.info('Exiting gracefully.')
    return 0

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
    

def substitute(template, vars):
    ''' Wrapper around Template.substitute for better error display. '''
    try:
        return Template(template).substitute(vars)
    except KeyError as e:
        msg = ('Error while substituting in string %r. Key %s not found: '
               'available keys are %s.' % (template, e, vars.keys()))
        raise Exception(msg)
    

def load_agent_and_state(agent_spec, id_agent, id_robot,
                         state_db_directory, log_directory, reset_state=False):
    ''' Load the agent, loading the agent state from the state_db directory.
        If the state is not available, then it initializes anew. The
        problem spec (sensel shape, commands shape) is loaded from the logs in log_directory. 
        
        Returns tuple agent, state.
    '''
    logger.info('Instancing agent spec:\n%s' % pformat(agent_spec))
    agent = instantiate_spec(agent_spec['code'])    
    db = LearningStateDB(state_db_directory)
    key = dict(id_robot=id_robot, id_agent=id_agent)
    if not reset_state and db.has_state(**key):
        logger.info('Using previous learned state.')
        state = db.get_state(**key)
        logger.info('State after learning %d episodes.' % len(state.id_episodes))
        try:
            agent.set_state(state.agent_state)
        except:
            logger.error('Could not set agent to previous state.')
            raise 
    else:
        state = LearningState(id_robot=id_robot, id_agent=id_agent)
        
        # Finding shape
        index = bag_get_index_object(log_directory)
        robots = index['robots']
        for ob0 in robots[id_robot][0].read():
            break
        
        sensel_shape = tuple(ob0.sensel_shape) # XXX
        commands_spec = eval(ob0.commands_spec)
        logger.info('Agent init Sensels: %s  commands: %s' % 
                    (sensel_shape, commands_spec))
        agent.init(sensel_shape, commands_spec)

    return agent, state

