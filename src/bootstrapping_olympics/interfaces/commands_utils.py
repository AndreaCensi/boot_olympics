import numpy as np

def random_command(command_spec):
    if isinstance(command_spec, tuple):
        lower, upper = command_spec
        return lower + np.random.rand() * (upper - lower)  
    elif isinstance(command_spec, list):
        n = len(command_spec)
        return command_spec[np.random.randint(n)]
    else:
        raise ValueError()
        
def random_commands(commands_spec):
    return np.array([random_command(x) for x in commands_spec])

def canonical_command(command_spec):
    i = np.random.randint(3)
    if i == 2:
        return 0
    else:
        return command_spec[i]        

def canonical_commands(commands_spec):
    return np.array([canonical_command(x) for x in commands_spec])

