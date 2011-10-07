from string import Template

def substitute(template, **kwargs):
    ''' Wrapper around Template.substitute for better error display. '''
    try:
        return Template(template).substitute(**kwargs)
    except KeyError as e:
        msg = ('Error while substituting in string %r. Key %s not found: '
               'available keys are %s.' % (template, e, kwargs.keys()))
        raise Exception(msg)
    
