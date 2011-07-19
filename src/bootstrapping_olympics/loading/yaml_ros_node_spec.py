from vehicles.configuration import BadConfig

def check_valid_ros_node_spec(x):
    if not isinstance(x, list):
        raise BadConfig(x, 'A ROS node spec must be a list with two elements.')
    
    if len(x) != 2:
        raise BadConfig(x, 'A ROS node spec must be a list of exactly two elements.')
    
    name = x[0]
    params = x[1]
    if not isinstance(name, str):
        raise BadConfig(x, 'The node must be given as a string.')
    if not '/' in name:
        msg = 'The node name must be in the format "<package>/<node>"'
        raise BadConfig(x, msg)
    if not isinstance(params, dict):
        raise BadConfig(x, 'The node params be given as a dictionary.')

def parse_yaml_ros_node_spec(x):
    check_valid_ros_node_spec(x)
    code = x[0]
    params = x[1]
    tokens = code.split('/')
    package_name = tokens[0]
    node_name = tokens[1]
    return package_name, node_name, params
