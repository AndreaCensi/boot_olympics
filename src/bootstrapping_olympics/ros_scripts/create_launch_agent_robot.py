import logging
from optparse import OptionParser
from ..loading import load_configuration, Configuration
from string import Template
from contracts.main import contract
from bootstrapping_olympics.loading.yaml_ros_node_spec import parse_yaml_ros_node_spec
import yaml



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
    
def create_launch_xml(id_agent, id_robot, namespace, bag=None):
    if not id_robot in Configuration.robots:
        msg = ('Robot ID %r not known.\nI know %s' % 
                (id_robot, Configuration.robots.keys()))
        raise Exception(msg)
    
    if not id_agent in Configuration.agents:
        msg = ('Agent ID %r not known.\nI know %s' % 
                (id_agent, Configuration.agents.keys()))
        raise Exception(msg)

    agent = Configuration.agents[id_agent]
    robot = Configuration.robots[id_robot]
    
    node_robot = create_ros_node_xml('my_robot', robot['ros-node'], remap={})
    node_agent = create_ros_node_xml('my_agent', agent['ros-node'],
                                     remap={
                                           'observations': 'my_robot/observations',
                                           'commands': 'my_robot/commands',
                                    })
    if bag is not None:
        node_bag = create_ros_node_xml('record', ['rosbag/record', {}],
                                        args="-a -o %s" % bag,
                                        output="screen")
    else:
        node_bag = ""
    
    
    template = """
<launch>
    <group ns="${namespace}" clear_params="true">
        <!-- Robot node -->
${node_robot}
        <!-- Agent node -->
${node_agent}
${node_bag}
    </group>
</launch>
""" 
    final = Template(template).substitute(node_robot=node_robot,
                node_agent=node_agent,
                namespace=namespace,
                node_bag=node_bag)
    return final


def create_ros_node_xml(node_name, ros_node, remap=None, args=None, output=None):
    
    package_name, node_type, params = parse_yaml_ros_node_spec(ros_node)
    
    xml_params = create_params_xml(params)
    xml_remap = create_remaps_xml(remap) if remap else ""
     
    template = """
        <node  pkg="${package_name}" 
              name="${node_name}" 
              type="${node_type}"   
              ${args}
              ${output}> 
${xml_params} 
${xml_remap}
        </node>
    """
    
    final = Template(template).substitute(
                package_name=package_name,
                node_name=node_name,
                node_type=node_type,
                xml_remap=xml_remap,
                xml_params=xml_params,
                args="args='%s'" % args if args else "",
                output="output='%s'" % output if output else "",)
    return final

def create_params_xml(params):
    return "\n".join([create_param_xml(name, value) 
                      for name, value in params.items()])

def create_param_xml(name, value):
    yaml_value = yaml.dump(value)
    return "\t\t\t<rosparam param='%s'>%s</rosparam>" % (name, yaml_value)

def create_remaps_xml(remap):
    return "\n".join([create_remap_xml(name, value) 
                      for name, value in remap.items()])

def create_remap_xml(sfrom, sto):
    return '\t\t\t<remap from="%s" to="%s"/>' % (sfrom, sto)


if __name__ == '__main__':
    main()
