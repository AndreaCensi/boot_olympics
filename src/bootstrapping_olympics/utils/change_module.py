import sys


def assign_all_to_module(module):
    for _, v in module.__dict__.items():
        try:
            # change if it is a child
            if module.__name__ in v.__module__:
                v.__module__ = module.__name__
        except:
            pass


def assign_all_symbols_to_module(module_name): 
    module = sys.modules[module_name]
    for _, v in module.__dict__.items():
        try:
            # change if it is a child
            if module.__name__ in v.__module__:
                v.__module__ = module.__name__
        except:
            pass
