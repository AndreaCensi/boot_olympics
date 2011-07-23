from bootstrapping_olympics.loading import (check_valid_ros_node_spec,
    parse_yaml_ros_node_spec)
from string import Template
import yaml

def create_launch_xml(agent_ros_node, robot_ros_node, namespace, bag=None, output=None):
    check_valid_ros_node_spec(agent_ros_node)
    check_valid_ros_node_spec(robot_ros_node)  
    node_robot = create_ros_node_xml('my_robot',
                                     robot_ros_node,
                                     remap={},
                                     output=output)
    node_agent = create_ros_node_xml('my_agent', agent_ros_node,
                                     remap={
                                           'observations': 'my_robot/observations',
                                           'commands': 'my_robot/commands',
                                    }, output=output)
    if bag is not None:
        node_bag = create_ros_node_xml('record', ['rosbag/record', {}],
                                        args="-a -o %s" % bag,
                                        output=output)
    else:
        node_bag = "<!-- bag disabled -->"
    
    
    template = \
"""<launch>
    <group ns="${namespace}" clear_params="true">
        <!-- Robot node -->
${node_robot}
        <!-- Agent node -->
${node_agent}
${node_bag}
    </group>
</launch>""" 
    final = Template(template).substitute(
                node_robot=indent(node_robot, ' '*8),
                node_agent=indent(node_agent, ' '*8),
                namespace=namespace,
                node_bag=indent(node_bag, ' '*8))
    return final


def create_ros_node_xml(node_name, ros_node, remap=None, args=None, output=None):
    package_name, node_type, params = parse_yaml_ros_node_spec(ros_node)
    
    xml_params = create_params_xml(params)
    xml_remap = create_remaps_xml(remap)
     
    template = \
"""<node pkg="${package_name}" type="${node_type}" name="${node_name}" ${output} ${args}> 
${xml_params} 
${xml_remap}
</node>"""
    final = Template(template).substitute(
                package_name=package_name,
                node_name=node_name,
                node_type=node_type,
                xml_remap=indent(xml_remap, ' '*4),
                xml_params=indent(xml_params, ' '*4),
                args="args='%s'" % args if args else "",
                output="output='%s'" % output if output else "",)
    return final

def indent(s, prefix):
    lines = s.split('\n')
    lines = ['%s%s' % (prefix, line) for line in lines]
    return '\n'.join(lines)


def create_params_xml(params):
    if not params: return '<!-- No parameters specified -->'
    return "\n".join([create_param_xml(name, value) 
                      for name, value in params.items()])

def create_param_xml(name, value):
    if isinstance(value, (int, float, str)):
        return "<rosparam param='%s'>%r</rosparam>" % (name, value)
    else:
        yaml_value = yaml.dump(value, default_flow_style=True).strip()
        return "<rosparam param='%s'>%s</rosparam>" % (name, yaml_value)

def create_remaps_xml(remap):
    if not remap: return '<!-- No remappings specified -->'
    return "\n".join([create_remap_xml(name, value) 
                      for name, value in remap.items()])

def create_remap_xml(sfrom, sto):
    return '<remap from="%s" to="%s"/>' % (sfrom, sto)

