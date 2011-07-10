import logging
from optparse import OptionParser 
from ..loading import load_configuration
import yaml
from bootstrapping_olympics.ros_scripts.launch_xml import create_launch_xml



logging.basicConfig();
logger = logging.getLogger("create_launch")
logger.setLevel(logging.DEBUG)


def main():
    parser = OptionParser()
    parser.add_option("-d", dest='directory',
                      help="Configuration directory.")
    parser.add_option("-a", "--agent", dest='agent',
                      help="ID of the agent.")
    parser.add_option("-r", "--robot", dest="robot",
                      help="ID of the robot.")
    parser.add_option("-o", "--output", dest="output",
                      help=".launch file")
    parser.add_option("-n", "--namespace", dest="namespace",
                      help="ROS namespace", default='test00')
    parser.add_option("-b", "--bag", dest="bag",
                      help="rosbag filename")
    (options, args) = parser.parse_args()

    if args: 
        raise Exception('Spurious arguments')
    if options.output is None:
        raise Exception('Please pass --output.')
    if options.agent is None:
        raise Exception('Please pass --agent.')
    if options.robot is None:
        raise Exception('Please pass --robot.')
    
    load_configuration(options.directory)
    
    create_launch(options.agent, options.robot, options.output,
                  namespace=options.namespace,
                  bag=options.bag)
    
    
def create_launch(id_agent, id_robot, output, namespace, bag):
    
    xml = create_launch_xml(id_agent, id_robot, namespace, bag)
    logger.info('Writing to file %r.' % output)
    with open(output, 'w') as f:
        f.write(xml)
    
if __name__ == '__main__':
    main()
