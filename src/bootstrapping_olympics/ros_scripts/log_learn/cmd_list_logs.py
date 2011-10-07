from . import check_no_spurious, logger
from optparse import OptionParser
import os

__all__ = ['cmd_list_logs']

def cmd_list_logs(data_central, argv):
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
    
    check_no_spurious(args)

    index = data_central.get_log_index(ignore_cache=options.refresh)
    
    
    logger.info('Index contains %d bag files with boot data.' % 
                len(index.file2streams))
    logger.info('In total, there are %d robots:' 
                % len(index.robot2streams))
    
    for robot, streams in index.robot2streams.items():
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
            logger.info('    total length: %.1f minutes' % 
                        (total_length / 60.0))
            logger.info('  total episodes: %d' % (total_episodes))
            logger.info('   total samples: %d' % (total_obs))
         
        if options.display_logs:
            for stream in streams:
                logger.info('- %5ds %s' % (stream.length, stream.bag_file))
#                                os.path.relpath(stream.bag_file,
#                                                main_options.log_directory)))
            
    if options.display_streams:
        for filename, streams in index.file2streams.items():
            if streams:
                logger.info('%s' % filename)
                                            
                for topic, content in streams.items():
                    logger.info('%s: %s' % (topic, content))
            else:
                logger.info('  No bootstrapping data found. ') 
 
cmd_list_logs.short_usage = 'list-logs [-v] [-vv]'


    
