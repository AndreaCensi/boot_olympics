

def assign_all_to_module(module): #, new_module_name=None):
#    if new_module is None:
#        new
    for _, v in module.__dict__.items():
        try:
            # change if it is a child
            if module.__name__ in v.__module__:
                v.__module__ = module.__name__
        except:
            pass
