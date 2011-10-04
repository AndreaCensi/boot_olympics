from conf_tools import check_generic_code_desc
                                      
def check_valid_robot_config(x):
    return check_generic_code_desc(x, 'robot')

#
#    necessary = [('id', str),
#                 ('desc', str),
#                 ('ros-node', list)]
#    check_necessary(x, necessary)
#    wrap_check(x, 'checking "code" entry',
#               check_valid_ros_node_spec, x['ros-node'])
