from bootstrapping_olympics.configuration import (check_valid_robot_config,
    check_valid_ros_node_spec, check_valid_agent_config)


def wrap_python_robot(robot_spec, root):
    check_valid_robot_config(robot_spec)
    node = ['bootstrapping_adapter/boot_robot_adapter.py',
                {'robot_spec': robot_spec,
                 'root': root}]
    check_valid_ros_node_spec(node)
    return node


def wrap_python_agent(agent_spec, root, publish_interval=0):
    check_valid_agent_config(agent_spec)
    node = ['bootstrapping_adapter/boot_agent_adapter.py',
                {'agent_spec': agent_spec,
                 'root': root,
                 'publish_interval': publish_interval}
         ]
    check_valid_ros_node_spec(node)
    return node


#def create_vehicles_launch(id_agent, id_vehicle, id_world, output_dir,
#                           log_level=0, viz_level=0, publish_interval=0):
#    logger.info('Creating launch for %s %s %s.' % (id_agent,
#  id_vehicle, id_world))
#
#    # We have to create a new 'robot'
#    id_robot = 'sim-%s-%s' % (id_agent, id_vehicle)
#    
#    vehicle = dict(**VehiclesConfig.vehicles[id_vehicle])
#    dereference_vehicle_spec(vehicle)
#    world = VehiclesConfig.worlds[id_world]
#    
#    simulation_code = ['vehicles_ros.ROSVehicleSimulation',
#                       {'vehicle': vehicle,
#                        'world': world,
#                        'viz_level': viz_level}]
#    
#    
#    agent = BootOlympicsConfig.agents[id_agent]
#    check_valid_agent_config(agent)
#    agent_node = ['bootstrapping_adapter/agent_adapter.py',
#                          {'code': agent['code'],
#                           'id_agent': id_agent,
#                           'publish_interval': publish_interval } ]

