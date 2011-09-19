from conf_tools import check_necessary

# TODO: unit tests         
def check_valid_event_config(x):
    necessary = [('id', str),
                 ('desc', str),
                 ('tasks', list),
                 ('agents', list),
                 ('robots', list)]
    check_necessary(x, necessary)
