from . import logger
from bootstrapping_olympics.loading import (Configuration as  BOConf,
    check_valid_agent_config)
from bootstrapping_olympics.ros_scripts import create_launch_xml
from vehicles.configuration import Configuration, dereference_vehicle_spec
import os
import shutil

def create_vehicles_launch(id_agent, id_vehicle, id_world, output_dir,
                           log_level=0, viz_level=0, publish_interval=0):
    logger.info('Creating launch for %s %s %s.' % (id_agent, id_vehicle, id_world))

    # We have to create a new 'robot'
    id_robot = 'sim-%s-%s' % (id_agent, id_vehicle)
    
    vehicle = dict(**Configuration.vehicles[id_vehicle])
    dereference_vehicle_spec(vehicle)
    world = Configuration.worlds[id_world]
    
    simulation_code = ['vehicles_ros.ROSVehicleSimulation',
                       {'vehicle': vehicle,
                        'world': world,
                        'viz_level': viz_level}]
    
    robot_node = ['bootstrapping_adapter/robot_adapter.py',
                          {'code': simulation_code } ]
    
    agent = BOConf.agents[id_agent]
    check_valid_agent_config(agent)
    agent_node = ['bootstrapping_adapter/agent_adapter.py',
                          {'code': agent['code'],
                           'id_agent': id_agent,
                           'publish_interval': publish_interval } ]
    
    if log_level > 0:
        # TODO: implement full/minimal
        bag_prefix = '%s-%s' % (id_vehicle, id_agent)
    else:
        bag_prefix = None
    
    output = 'screen'
    xml = create_launch_xml(agent_node, robot_node,
                            namespace='boot_olympics', bag=bag_prefix,
                            output=output)
    logger.info('Launch configuration:\n%s' % xml) 
    
    basename = '%s.launch' % id_robot
    output = os.path.join(output_dir, 'boot_gui_gen/vsim/%s' % basename)
    logger.info('Writing to file %r.' % output)
    dirname = os.path.dirname(output)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
        
    with open(output, 'w') as f:
        f.write(xml)
    
    
    logger.info('Created launch file in %r.' % basename)

    other = 'last.launch'
    output2 = os.path.join(output_dir, other)
    shutil.copyfile(output, output2)
    logger.info(' (also available as %r)' % other)
