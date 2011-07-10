from bootstrapping_olympics.loading.yaml_ros_node_spec import parse_yaml_ros_node_spec
from string import Template
import yaml
from bootstrapping_olympics.loading.load_all import Configuration

def create_launch_xml(id_agent, id_robot, namespace, bag=None, output=None):
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
    
    node_robot = create_ros_node_xml('my_robot', robot['ros-node'], remap={},
                                     output=output)
    node_agent = create_ros_node_xml('my_agent', agent['ros-node'],
                                     remap={
                                           'observations': 'my_robot/observations',
                                           'commands': 'my_robot/commands',
                                    }, output=output)
    if bag is not None:
        node_bag = create_ros_node_xml('record', ['rosbag/record', {}],
                                        args="-a -o %s" % bag,
                                        output=output)
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

