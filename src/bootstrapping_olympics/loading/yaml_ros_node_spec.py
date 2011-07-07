def parse_yaml_ros_node_spec(x):
    assert isinstance(x, list)
    assert len(x) == 2
    code = x[0]
    params = x[1]
    assert '/' in code
    tokens = code.split('/')
    package_name = tokens[0]
    node_name = tokens[1]
    return package_name, node_name, params

