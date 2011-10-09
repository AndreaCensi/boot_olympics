from . import logger
from bootstrapping_olympics.utils import wrap_script_entry_point
from optparse import OptionParser
import os
import tables

usage = """

    boot_olympics_rosbag2h5 <directory>
    
Creates a .h5 file for each .bag file.
  
"""  

def rosbag2h5(pargs):
    parser = OptionParser(usage=usage)
    parser.disable_interspersed_args()
    parser.add_option("-l", dest='log_directory',
                      default="~/.ros/log", # XXX: log
                      help="Log directory [%default].")

    (options, args) = parser.parse_args()
    
    if args:
        msg = 'Spurious arguments.'
        raise Exception(msg)

    index = LogIndex()
    index.index(options.log_directory)

    logger.info('Found %s files.' % len(index.file2streams))
    for filename, streams in index.file2streams.items():
        if len(streams) != 1:
            msg = ('Cannot deal with %d streams per file %r.' % 
                   (len(streams), filename)) 
            logger.error(msg)
            continue
        convert(streams[0]) 
    
def convert(stream):
    filename = stream.bag_file
    h5 = os.path.splitext(filename)[0] + '.h5'
    logger.info('Creating %r' % h5)
    f = tables.openFile(h5, 'w')
    
    # XXX: to finish
    f.close() 

def main(): 
    wrap_script_entry_point(rosbag2h5, logger)

if __name__ == '__main__':
    main()
