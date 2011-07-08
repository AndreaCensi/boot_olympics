from optparse import OptionParser
import itertools
import logging
import os

from ..loading import load_configuration, Configuration
from .create_launch_agent_robot import create_launch_xml

logging.basicConfig();
logger = logging.getLogger("create_launch_all")
logger.setLevel(logging.DEBUG)


def main():
    parser = OptionParser()
    parser.add_option("-d", dest='directory',
                      help="Configuration directory.")
    parser.add_option("-o", "--output", dest="output",
                      help="destination directory")
    parser.add_option("-n", "--namespace", dest="namespace",
                      help="ROS namespace", default='boot_olympics')
    parser.add_option("-b", "--bag", dest="bag",
                      help="rosbag filename")
    (options, args) = parser.parse_args()

    if args: 
        raise Exception('Spurious arguments')
    if options.output is None:
        raise Exception('Please pass --output.')
    
    load_configuration(options.directory)
    
    if not os.path.exists(options.output):
        os.makedirs(options.output)
        
    for id_robot, id_agent in itertools.product(Configuration.robots, Configuration.agents):
    
        process_output = "screen"
        xml = create_launch_xml(id_agent=id_agent, id_robot=id_robot,
                            namespace=options.namespace,
                            output=process_output)
    
        basename = '%s-%s.launch' % (id_robot, id_agent)
        filename = os.path.join(options.output, basename)
        logger.info('Writing to file $DIR/ %r.' % basename)
        with open(filename, 'w') as f:
            f.write(xml)
    

if __name__ == '__main__':
    main()
