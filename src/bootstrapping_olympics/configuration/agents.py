from conf_tools import check_generic_code_desc


def check_valid_agent_config(x):
    return check_generic_code_desc(x, 'agent')


def check_valid_plugin_config(x):
    return check_generic_code_desc(x, 'plugin')
