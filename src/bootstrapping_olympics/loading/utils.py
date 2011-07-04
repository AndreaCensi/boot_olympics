from contracts.main import contract, check
#import collections


@contract(code_spec='seq[2]')
def instantiate_spec(code_spec):
    ''' code_spec must be a sequence  [string, dictionary], giving
        the python function (or class) to instantiate, along
        with its parameters. '''
    function_name = code_spec[0]
    parameters = code_spec[1]
    check('str', function_name)
    check('dict', parameters)
    
    function = import_name(function_name)

#    if not isinstance(collections.Callable, function):
#        raise Exception('Cannot call %r (%s)' % (function_name, function))
    try:
        return function(**parameters)
    except TypeError as e:
        msg = ('Could not instantiate [%r, %s]:\n\t%s' % 
               (function_name, parameters, e))
        raise Exception(msg) 
    

@contract(name='str')
def import_name(name):
    ''' 
        Loads the python object with the given name. 
    
        Note that "name" might be "module.module.name" as well.
    '''
    try:     
        return __import__(name, fromlist=['dummy'])
    except ImportError as e:
        # split in (module, name) if we can
        if '.' in name:
            tokens = name.split('.')
            field = tokens[-1]
            module_name = ".".join(tokens[:-1])
            
            try: 
                module = __import__(module_name, fromlist=['dummy'])
            except ImportError as e:
                msg = 'Cannot load %r (tried also with %r): %s.' % (name, module_name, e)
                raise Exception(msg)
            
            if not field in module.__dict__:
                msg = 'No field %r found in module %r.' % (field, module)
                raise Exception(msg) 
            
            return module.__dict__[field]
        else:
            msg = 'Cannot import name %r, and cannot split: %s' % (name, e)
            raise Exception(msg)
    
