from . import logger, contract, np
from contracts import check, describe_type, describe_value
import warnings
from numpy.core.numeric import allclose
from numpy.testing.utils import assert_allclose


class ValueFormats:
    Continuous = 'C' # range is [min, max]
    Discrete = 'D' # finite number of elements 
    Invalid = 'I' # invalid/not used # TODO: tests for invalid values
    valid = [Continuous, Discrete, Invalid]

streamel_dtype = [('kind', 'S1'), # 'I','D','C'
                  ('lower', 'float'),
                  ('upper', 'float')]

@contract(streamels='array')
def check_valid_streamels(streamels):
    assert streamels.dtype == np.dtype(streamel_dtype)
    # TODO: check shape 
    # TODO: check bounds 

class StreamSpec:

    # TODO: make this accept only the a streamel_dtype array, put the interpretation
    # in another module    
    @contract(id_stream='None|str', streamels='array',
              extra='None|dict', filtered='None|dict', desc='None|str')
    def __init__(self, id_stream, streamels, extra, #@ReservedAssignment
                 filtered=None, desc=None): #@ReservedAssignment
        
        self.id_stream = id_stream
        self.desc = desc
        self.extra = extra
        self.filtered = filtered

        check_valid_streamels(streamels)
        self.streamels = streamels        
        self.kind = self.streamels['kind']
        self.lower = self.streamels['lower']
        self.upper = self.streamels['upper']
        
    def __eq__(self, other):
        return np.all(self.streamels == other.streamels)
    
    @staticmethod
    def check_same_spec(spec1, spec2):
        s1 = spec1.get_streamels()
        s2 = spec2.get_streamels()
        assert np.all(s1['kind'] == s2['kind'])
        assert_allclose(s1['lower'], s2['lower'])
        assert_allclose(s1['upper'], s2['upper'])

    def get_streamels(self):
        return self.streamels.copy()
     
    @contract(returns='array')
    def get_default_value(self):
        ''' Returns a "default value" for this stream. For
            commands streams, this has the semantics of being
            a "at rest" value. For observations, this is an "example"
            value, used for visualization. '''
        warnings.warn("Values set to zero and not loaded from config.")
        return np.zeros(self.streamels.shape)
    
    @contract(returns='array')
    def get_random_value(self):
        ''' Returns a "random value" for this stream. 
            This is distributed uniformly in the ranges. 
            (representation dependent)'''
        x = np.zeros(self.streamels.shape)
        
        streamelsf = self.streamels.flat
        for i in xrange(x.size):
            lower = streamelsf[i]['lower']
            upper = streamelsf[i]['upper']
            kind = streamelsf[i]['kind'] 
            if kind == ValueFormats.Continuous:
                val = np.random.uniform(lower, upper)
            elif kind == ValueFormats.Discrete:
                val = np.random.randint(lower, upper)
            elif kind == ValueFormats.Invalid:
                val = np.NaN
            else: assert False
            x.flat[i] = val
        return x
    
    @contract(x='array')
    def check_valid_value(self, x):
        ''' Checks if the value x is valid according to this spec
            and returns ValueError if it is not. '''
        def bail(msg):
            msg += '\n  stream: %s' % self
            msg += '\n   value: %s' % describe_value(x)
            raise ValueError(msg)
        def display_some(x, select):
            ''' Displays some of the elements in x (array) given
                by select (array of bool). '''
            how_many = np.sum(select)
            to_display = min(how_many, 4)
            s = 'First %s of %s/%s: %s' % (to_display, how_many, x.size,
                                           x[select][:to_display])
            return s
        
        if not isinstance(x, np.ndarray):
            msg = 'Type is %s instead of numpy array.' % describe_type(x)
            bail(msg)
            
        if x.shape != self.streamels.shape:
            msg = ('Expected shape %s instead of %s.' % 
                   (self.streamels.shape, x.shape))
            bail(msg)
        
        invalids = self.kind == ValueFormats.Invalid
        valid = np.logical_not(invalids)
        # continuous = self.kind == ValueFormats.Continuous 
        discrete = self.kind == ValueFormats.Discrete
        
        # First check all invalids are NaN
        if not np.all(np.isnan(x[invalids])): # XXX: slow
            msg = 'Not all invalids are set to NaN.'
            raise ValueError(msg)
        
        # Check that all valid are not NaN
        xv = x[valid]
        xv_nan = np.isnan(xv)
        if np.any(xv_nan):
            msg = ('Found NaNs in the valid values. %s' % 
                    display_some(xv, xv_nan))
            bail(msg)
         
        # Check all discrete values are integers
        d = x[discrete]
        not_discrete = np.round(d) != d
        if np.any(not_discrete):
            msg = ('Not all discrete are integers. %s' % 
                   display_some(d, not_discrete))
            bail(msg)
        
        lv = self.lower[valid]
        uv = self.upper[valid]
        out_of_range = np.logical_or(xv < lv, xv > uv)
        if np.any(out_of_range):
            msg = ('Found elements out of range. %s' % 
                   display_some(xv, out_of_range))
            bail(msg) 
    
    @contract(returns='int,>0')
    def size(self):
        ''' Returns the number of total elements in the stream. '''
        return self.streamels.size
    
    @contract(returns='seq[>0](int,>0)')
    def shape(self):
        ''' Returns the shape of the stream, as a sequence of positive 
            integers (like Numpy's shape). '''
        return self.streamels.shape

    @staticmethod
    @contract(spec='dict')
    def from_yaml(spec):
        try:
            if not isinstance(spec, dict):
                raise ValueError('Expected a dict, got %s' % spec)
            s = dict(**spec)
            id_stream = s.pop('id', None)
            desc = s.pop('desc', None)
            required = ['shape', 'format', 'range']
            for x in required:
                if not x in s:
                    raise ValueError('Missing entry %r.' % x)
            shape = s.pop('shape')
            check('list[>0](int,>0)', shape)
            format = s.pop('format') #@ReservedAssignment
            range = s.pop('range') #@ReservedAssignment
            extra = s.pop('extra', {})
            filtered = s.pop('filtered', None)
            
            if s.keys():
                logger.warning('While reading\n%s\nextra keys detected: %s' % 
                               ((spec), s.keys()))
            
            streamels = streamels_from_spec(shape, format, range)
            
            return StreamSpec(id_stream, streamels, extra, filtered, desc)
        except:
            logger.info('Error while parsing the StreamSpec:\n%s' % spec)
            raise
        
    @contract(returns='dict')
    def to_yaml(self):
        arange = []
        if self.streamels.ndim == 1:
            for i in range(self.streamels.size):
                arange.append(get_streamel_range(self.streamels[i]))
        elif self.streamels.ndim == 2:
            for i in range(self.streamels.shape[0]):
                row = []
                for j in range(self.streamels.shape[1]):
                    row.append(get_streamel_range(self.streamels[i, j]))
                arange.append(row)
        else: assert False
            
        data = {
            'shape': list(self.streamels.shape),
            'format': self.kind.tolist(),
            'range': arange,
            'id': self.id_stream,
            'desc': self.desc,
            'extra': self.extra,
            'filtered': self.filtered        
        }
        return data

    def __str__(self):
        return 'StreamSpec(%s)' % str(self.streamels.shape)
    
    @staticmethod
    def join(obs1, obs2):
        ''' Returns the spec obtained by concatenating two of them. '''
        assert isinstance(obs1, StreamSpec)
        assert isinstance(obs2, StreamSpec)
        if len(obs1.streamels.shape) != 1:
            raise Exception('Not implemented for 2D signals')
        if len(obs2.streamels.shape) != 1:
            raise Exception('Not implemented for 2D signals')
        joint = np.hstack((obs1.streamels, obs2.streamels))
        assert len(joint.shape) == 1
        
        # XXX: this can be done better
        # names TODO
        id_stream = '%s-%s' % (obs1.id_stream, obs2.id_stream)
        shape = list(joint.shape)
        format = joint['kind'].tolist() #@ReservedAssignment
        mrange = np.zeros(shape=joint.shape + (2,)) # XXX: use int?
        mrange[..., 0] = joint['lower']
        mrange[..., 1] = joint['upper']
        mrange = mrange.tolist()
        extra = {}
        filtered = dict(filter='join', original=[obs1.to_yaml(), obs2.to_yaml()])
        desc = 'Join of %s and %s' % (obs1.id_stream, obs2.id_stream)  #@UnusedVariable
        
        streamels = streamels_from_spec(shape, format, mrange)
        
        return StreamSpec(id_stream=id_stream,
                          streamels=streamels, extra=extra,
                          filtered=filtered, desc=desc)

    
def check_valid_bounds(bounds):
    if not isinstance(bounds, (list, np.ndarray)):
        msg = 'Expect list or array, got %s.' % bounds
        raise ValueError(msg) 
    expect_size(bounds, 2) 
    if not bounds[0] < bounds[1]:
        raise ValueError('Invalid bounds lower: %s upper: %s (lower>=upper)' % 
                         (bounds[0], bounds[1]))

def expect_one_of(x, options):
    if not x in options:
        msg = ('Got %r, expected one of %s' % (x, options))
        raise ValueError(msg)


def expect_size(x, l):
    if not len(x) == l:
        msg = 'Expected len %s, got %s.' % (l, len(x))
        raise ValueError(msg)


def set_streamel_range(streamel, bounds):
    if streamel['kind'] == ValueFormats.Invalid:
        if (bounds is not None) and (bounds != [None, None]):
            msg = ('Incorrect bounds spec for invalid streamel: %r (%s).' 
                   % (bounds, (bounds is None)))
            raise ValueError(msg)
        streamel['lower'] = np.inf # do not use nan, otherwise streamels!=streamels
        streamel['upper'] = np.inf
    else:
        check_valid_bounds(bounds)
        streamel['lower'] = bounds[0]
        streamel['upper'] = bounds[1]
        
def get_streamel_range(streamel):
    if streamel['kind'] == ValueFormats.Invalid:
        return None
    else:
        return [float(streamel['lower']), float(streamel['upper'])] 
    

@contract(shape='list[int]', format='list|str', range='list',
          returns='array')
def streamels_from_spec(shape, format, range): #@ReservedAssignment
    streamels = np.zeros(shape=shape, dtype=streamel_dtype)
    kind = streamels['kind']
    lower = streamels['lower']
    upper = streamels['upper']
     
    # If the "format" is a string it is valid for all of them
    if isinstance(format, str):
        expect_one_of(format, ValueFormats.valid)
        kind.flat[:] = format
        # at this point the range must be unique
        check_valid_bounds(range)
        lower.flat[:] = range[0]
        upper.flat[:] = range[1]
    else:
        # If the format is not a string, then it must be a list
        if not isinstance(format, list):
            msg = 'Expected list for "format", got %s.' % describe_value(format)
            raise ValueError(msg)

        # And it must have the same number of elements
        formats = np.array(format)
        if formats.shape != streamels.shape:
            msg = ('Expected format shape to be %s instead of %s.' % 
                   (formats.shape, streamels.shape)) 
            raise ValueError(msg)
        for i in xrange(formats.size):
            expect_one_of(formats.flat[i], ValueFormats.valid)
            kind.flat[i] = formats.flat[i]
            
        # Also the range must be a list
        if not isinstance(range, list):
            msg = 'Expected list for "range", got %s.' % describe_value(range)
            raise ValueError(msg)

        if len(streamels.shape) == 1:
            if len(range) != streamels.shape[0]:
                raise ValueError('Expected %s, got %s.' % 
                                 (streamels.shape[0], len(range))) 
                
            for i in xrange(streamels.shape[0]):
                set_streamel_range(streamels[i], range[i])

        elif len(streamels.shape) == 2:
            if len(range) != streamels.shape[0]:
                raise ValueError('Expected %s, got %s.' % 
                                 (streamels.shape[0], len(range))) 
            for i in xrange(streamels.shape[0]):
                if len(range[i]) != streamels.shape[1]:
                    raise ValueError('Expected %s, got %s.' % 
                                      (streamels.shape[1], len(range[i])))
                for j in xrange(streamels.shape[1]):
                    set_streamel_range(streamels[i, j], range[i][j])
        else:
            raise ValueError('Not implemented for shape %s.' % 
                             str(streamels.shape))
    return streamels
