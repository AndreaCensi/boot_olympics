from . import contract
from .. import BootstrappingObservations
from bootstrapping_olympics.utils import yaml_dump_inline


class Cache:
    last_robot_spec = None
    last_spec = None


@contract(obs='array')
def observations2ros(robot_spec, obs):
    ''' 
        Converts the array format used internally in the Python program
        to a ROS message. The ROS message also includes the robot spec.
        
        ROS has some problems with arrays of different dimensions, so the
        data is flattened.
 
        Spec always included
        
        Other quirks (to be changed in the future):
        - id_environment instead of id_world 
    '''

    if id(Cache.last_robot_spec) != id(robot_spec):
        Cache.last_spec = yaml_dump_inline(robot_spec.to_yaml())

    fields = {
        'timestamp': obs['timestamp'],
        'sensel_values': obs['observations'].flatten().tolist(),
        'sensel_shape': obs['observations'].shape,
        'commands': obs['commands'].tolist(),
        'commands_source': obs['commands_source'].item(),

        'counter': obs['counter'],
        'id_episode': obs['id_episode'].item(),
        'id_environment': obs['id_world'].item(),

        'id_robot': obs['id_robot'].item(),
        'id_actuators': robot_spec.get_observations().get_id_stream(),
        'id_sensors': robot_spec.get_commands().get_id_stream(),
        'commands_spec': Cache.last_spec,
        'type': 'sim' # XXX: do we need this at all?
        # XXX: what about the 'version' field?
    }
    return BootstrappingObservations(**fields)

