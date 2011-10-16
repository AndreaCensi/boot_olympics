from . import check_mandatory, check_no_spurious, logger
from optparse import OptionParser

def cmd_video(data_central, argv):
    '''Runs the learning for a given agent and log. ''' 
    parser = OptionParser(usage=cmd_video.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-a", "--agent", dest='agent', help="Agent ID")
    parser.add_option("-r", "--robot", dest='robot', help="Robot ID") 
    parser.add_option("-z", "--zoom", dest='zoom', type='float',
                      default=0,
                       help="Zoom in m (0: whole map)") 
    parser.add_option("-m", "--model", dest='model', default='boot_log2movie',
                      help="Procgraph model used for visualization.") 
    
    (options, args) = parser.parse_args(argv)
    
    check_no_spurious(args)
    check_mandatory(options, ['agent', 'robot'])
    
    create_video(data_central=data_central,
              id_agent=options.agent,
              id_robot=options.robot,
              zoom=options.zoom,
              model=options.model)
    
cmd_video.short_usage = "video -a <agent> -r <robot> [-z <zoom>]"


def create_video(data_central, id_robot, id_agent, model='boot_log2movie', zoom=0):
    logger.info('Creating video for robot: %r agent: %r zoom: %s' % 
                (id_robot, id_agent, zoom))
    
    ds = data_central.get_dir_structure()
    basename = ds.get_video_basename(id_robot, 'a%s' % id_agent)
    if zoom:
        basename += '-z%.1f' % zoom
    
    filename = basename + '-%s.avi' % model
    logdir = ds.get_simulated_logs_dir()
    import procgraph_vehicles #@UnusedImport
    from procgraph import pg
    logger.info('Writing to %r.' % filename)
    config = dict(logdir=logdir,
                id_robot=id_robot,
                id_agent=id_agent,
                id_episode='',
                zoom=zoom,
                output=filename)
    pg(model, config=config)
