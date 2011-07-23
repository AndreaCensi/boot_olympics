from . import logger
from optparse import OptionParser 
import os 
from . import bag_get_index_object

usage = """
    
    
    list-logs     Shows every log
    

"""
def cmd_list_logs(main_options, argv):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
#    parser.add_option("-d", dest='conf_directory',
#                      help="Configuration directory [%default].")
    parser.add_option("-s", "--streams", dest='display_streams', action='store_true',
                      default=False,
                      help="Displays all streams [%default].")
    (options, args) = parser.parse_args(argv)
    
    if args:
        raise Exception('Spurious arguments %s' % args)    
    
    index = bag_get_index_object(main_options.log_directory)
    bag_files = index['bag_files']
    robots = index['robots']
    logger.info('Index contains %d bag files with boot data.' % len(bag_files))
    

    for robot, streams in robots.items():
        logger.info('Robot %s has %d streams.' % (robot, len(streams)))
        
        
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

    
