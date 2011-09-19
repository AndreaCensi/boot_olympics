from ..logs import bag_get_index_object

from . import  logger
from optparse import OptionParser
import os

__all__ = ['cmd_list_logs']

def cmd_list_logs(main_options, argv):
    '''Shows information about every log. '''
    
    parser = OptionParser(usage=cmd_list_logs.short_usage)
    parser.disable_interspersed_args()
    parser.add_option("-R", dest='refresh', action='store_true',
                      default=False,
                      help="Ignores global cache.")
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
    
    index = bag_get_index_object(main_options.log_directory,
                                 ignore_cache=options.refresh)
    bag_files = index['bag_files']
    robots = index['robots']
    logger.info('Index contains %d bag files with boot data.' % len(bag_files))
    
    logger.info('In total, there are %d robots:' % len(robots))
    for robot, streams in robots.items():
        logger.info('- robot %r has %d streams.' % (robot, len(streams)))
        if streams:
            logger.info('  spec: %s' % streams[0].spec)
            total_length = 0
            total_obs = 0
            total_episodes = 0
            for stream in streams:
                total_length += stream.length
                total_obs += stream.num_observations
                total_episodes += len(stream.id_episodes)
            logger.info('    total length: %.1f minutes' % (total_length / 60.0))
            logger.info('  total episodes: %d' % (total_episodes))
            logger.info('   total samples: %d' % (total_obs))
         
        if options.display_logs:
            for stream in streams:
                logger.info('- %5ds %s' % (stream.length,
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

cmd_list_logs.short_usage = 'list-logs [-v] [-vv]'

    


    
