'''
    A streamel (stream element) describes the *format* of the data stream.
    It specifies what are the valid values that the data can take. 

'''

from . import contract, np
from ..utils import indent, show_differences
from contracts import  new_contract
from bootstrapping_olympics.interfaces import BOOT_OLYMPICS_SENSEL_RESOLUTION

__all__ = ['ValueFormats', 'streamel_dtype', 'new_streamels',
           'streamel_array', 'check_valid_streamels', 'make_streamels_2D_float']


class ValueFormats:
    Continuous = 'C'  # range is [lower, upper]
    Discrete = 'D'  # finite number of elements 
    Invalid = 'I'  # invalid/not used # TODO: tests for invalid values
    valid = [Continuous, Discrete, Invalid]

streamel_dtype = [
      ('kind', 'S1'),  # 'I','D','C'
      ('lower', BOOT_OLYMPICS_SENSEL_RESOLUTION),  # This must be a finite value.
      ('upper', BOOT_OLYMPICS_SENSEL_RESOLUTION),  # This must be a finite value.
      ('default', BOOT_OLYMPICS_SENSEL_RESOLUTION)  # This must respect the bounds.
]


@contract(shape='(int,>0)|seq[>0](int,>0)')
def new_streamels(shape):
    ''' 
        Creates a new array of streamels, initialized with all invalid
        values. This is useful to check if we forgot to initialize 
        some fields.
    '''
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = '?'
    x['lower'] = np.nan
    x['upper'] = np.nan
    x['default'] = np.nan
    return x

@contract(shape='seq[2](int,>=1)', lower='scalar_number,finite,x',
          upper='scalar_number,finite,y,>x', vdef='None|(scalar_number,finite,>x,<y)')
def make_streamels_2D_float(shape, lower, upper, vdef=None):
    """ Creates a stremeals spec with the given shape, min and max. """
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x

@contract(shape='seq[>=1](int,>=1)', lower='scalar_number,finite,x',
          upper='scalar_number,finite,y,>x', vdef='None|(scalar_number,finite,>x,<y)')
def make_streamels_float(shape, lower, upper, vdef=None):
    """ Creates a stremeals spec with the given shape, min and max. """
    x = np.zeros(shape, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x

    
@contract(nstreamels='int,>=1',
          lower='scalar_number,finite,x',
          upper='scalar_number,finite,y,>x',
          vdef='None|(scalar_number,finite,>x,<y)')
def make_streamels_1D_float(nstreamels, lower, upper, vdef=None):
    x = np.zeros(nstreamels, streamel_dtype)
    x['kind'] = ValueFormats.Continuous
    x['lower'] = lower
    x['upper'] = upper
    if vdef is None: 
        vdef = lower * 0.5 + upper * 0.5
    x['default'] = vdef
    return x
    


@new_contract
@contract(x='array')
def streamel_array(x):
    ''' 
        Checks that the argument is a non-empty array of streamels. 
        This checks only the data type, not the validity of the data.
    '''
    # TODO: register_dtype(x, contract)
    if x.dtype != streamel_dtype:
        msg = 'Expected streamels array, obtained: %s.' % str(x.dtype)
        raise ValueError(msg)


@contract(streamels='streamel_array')
def check_valid_streamels(streamels):
    ''' 
        Raises a ValueError if the data in the structure is not coherent.
        
        This checks, for example, that the default value given respects
        the declared upper/lower bounds. 
    '''
    try:
        # XXX: how about invalid values?
        def check(which, msg, a, b):
            if not np.all(which):
                error = show_differences(a, b,
                                         is_failure=(~which), condition=msg)
                raise ValueError(error)

        not_discrete = streamels['kind'] != ValueFormats.Discrete
        invalid = streamels['kind'] == ValueFormats.Invalid

        check(np.logical_or(invalid,
                            streamels['lower'] < streamels['upper']),
              'lower<upper', streamels['lower'], streamels['upper'])
        check(np.logical_or(invalid,
                            streamels['default'] <= streamels['upper']),
              'default<=upper', streamels['default'], streamels['upper'])
        check(np.logical_or(invalid,
                            streamels['lower'] <= streamels['default']),
              'default<=upper', streamels['lower'], streamels['default'])

        check(np.logical_or(not_discrete,
            np.round(streamels['default']) == streamels['default']),
                            'default is discrete',
                            np.round(streamels['default']),
                            streamels['default'])
    except Exception as e:
        msg = ('This streamel specification is not coherent:\n'
                '%s\n'
                'streamels: %s' % 
                (indent(str(e), '>'), streamels))
        raise ValueError(msg)

