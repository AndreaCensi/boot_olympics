from .yaml_ros_node_spec import check_valid_ros_node_spec
from conf_tools import wrap_check, check_necessary

                                                     
def check_valid_robot_config(x):
    necessary = [('id', str),
                 ('desc', str),
                 ('ros-node', list)]
    check_necessary(x, necessary)
    wrap_check(x, 'checking "code" entry',
               check_valid_ros_node_spec, x['ros-node'])
