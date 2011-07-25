from . import logger
from optparse import OptionParser 
import os 
from . import bag_get_index_object
 

def cmd_list_logs(main_options, argv):
    '''Shows information about every log. '''
    
    parser = OptionParser(usage=cmd_list_logs.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-v", dest='display_logs', action='store_true',
                      default=False,
                      help="Displays all logs.")
    parser.add_option("--vv", dest='display_streams', action='store_true',
                      default=False,
                      help="Displays all streams.")
    (options, args) = parser.parse_args(argv)
    
    if args: 
        logger.error('Spurious args (%s)' % args)
        return -2 
    
    index = bag_get_index_object(main_options.log_directory)
    bag_files = index['bag_files']
    robots = index['robots']
    logger.info('Index contains %d bag files with boot data.' % len(bag_files))
    
    logger.info('In total, there are %d robots:' % len(robots))
    for robot, streams in robots.items():
        logger.info('- robot %r has %d streams.' % (robot, len(streams)))
        
        if options.display_logs:
            for stream in streams:
                logger.info('- %5d min %s' % (stream.length,
                                          os.path.relpath(stream.bag_file,
                                                          main_options.log_directory)))

    if options.display_streams:
        for bag_file, streams in bag_files.items():
            if streams:
                logger.info('%s' % os.path.relpath(bag_file, main_options.log_directory))
                for topic, content in streams.items():
                    logger.info('%s: %s' % (topic, content))
            else:
                logger.info('  No bootstrapping data found. ') 

    return 0

cmd_list_logs.short_usage = 'list-logs [-v]'

    


    
