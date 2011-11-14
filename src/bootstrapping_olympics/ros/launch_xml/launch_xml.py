from ...configuration.yaml_ros_node_spec import check_valid_ros_node_spec
from bootstrapping_olympics.ros.launch_xml.ros_xml_utils import (
    create_ros_node_xml)
from bootstrapping_olympics.utils.strings import indent
from string import Template

def create_launch_xml(agent_ros_node,
                      robot_ros_node,
                      agent_node_id='my_agent',
                      robot_node_id='my_robot',
                      namespace='boot_olympics',
                      bag=None,
                      output=None):
    '''
        output: None or 'screen'
        bag: None or name of the file
    '''
    check_valid_ros_node_spec(agent_ros_node)
    check_valid_ros_node_spec(robot_ros_node)  
    
    node_robot = create_ros_node_xml(robot_node_id,
                                     robot_ros_node,
                                     remap={},
                                     output=output)
    remap = {
        'observations': '%s/observations' % robot_node_id,
        'commands': '%s/commands' % robot_node_id
    }
            
    node_agent = create_ros_node_xml(agent_node_id, agent_ros_node,
                                     remap=remap, output=output)
    
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
                node_robot=indent(node_robot, ' ' * 8),
                node_agent=indent(node_agent, ' ' * 8),
                namespace=namespace,
                node_bag=indent(node_bag, ' ' * 8))
    return final


