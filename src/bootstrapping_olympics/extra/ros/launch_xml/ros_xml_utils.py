from bootstrapping_olympics.configuration import parse_yaml_ros_node_spec
from bootstrapping_olympics.utils import indent, yaml_dump_inline
from string import Template


def create_ros_node_xml(node_name, ros_node, remap=None,
                        args=None, output=None):
    package_name, node_type, params = parse_yaml_ros_node_spec(ros_node)

    xml_params = create_params_xml(params)
    xml_remap = create_remaps_xml(remap)

    template = \
"""<node pkg="${package_name}"
         type="${node_type}" 
         name="${node_name}" ${output} ${args}> 
${xml_params} 
${xml_remap}
</node>"""
    final = Template(template).substitute(
                package_name=package_name,
                node_name=node_name,
                node_type=node_type,
                xml_remap=indent(xml_remap, ' ' * 4),
                xml_params=indent(xml_params, ' ' * 4),
                args="args='%s'" % args if args else "",
                output="output='%s'" % output if output else "",)
    return final


def create_params_xml(params):
    if not params:
        return '<!-- No parameters specified -->'
    return "\n".join([create_param_xml(name, value)
                      for name, value in params.items()])


def create_param_xml(name, value):
    if isinstance(value, (int, float, str)):
        return "<rosparam param='%s'>%r</rosparam>" % (name, value)
    else:
        yaml_value = yaml_dump_inline(value).strip()
        return "<rosparam param='%s'>%s</rosparam>" % (name, yaml_value)


def create_remaps_xml(remap):
    if not remap:
        return '<!-- No remappings specified -->'

    def create_remap_xml(sfrom, sto):
        return '<remap from="%s" to="%s"/>' % (sfrom, sto)

    return "\n".join([create_remap_xml(name, value)
                      for name, value in remap.items()])

