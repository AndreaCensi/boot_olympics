from contracts import check, contract
import numpy as np
from . import logger

# Assumptions
#class Semantics:
#    Zero = '0'
#    Continuity = 'C'
#    Monoticity = 'M'
#    #PositiveSaliency = 'P'
#    #NegativeSaliency = 'P'
#    Symmetry = 'S'
#    Linearity = 'L'
#    
#    valid = [Zero, Continuity, Monoticity, Symmetry, Linearity]

class ValueFormats:
    Continuous = 'C' # range is [min, max]
    Discrete = 'D' # finite number of elements 
    Invalid = 'I' # invalid/not used TODO tests
#    Float = 'f' #  all float values
#    Positive = 'p' # all positive values
#    ZeroOne = '0' # [0,1]
#    OneMinusOne = '1' # [-1,+1]
#    Binary = '2' # {0, 1}
#    Ternary = '3' # {-1,0,1}
#    valid = [Float, Positive, ZeroOne, OneMinusOne, Binary, Ternary, Discrete, Continuous]
    valid = [Continuous, Discrete, Invalid]


streamel_dtype = [('kind', 'S1'), # 'I','D','C'
                ('lower', 'float'),
                ('upper', 'float')]

class StreamSpec:
    
    @contract(id_stream='None|str', shape='list[int]', format='list|str', range='list',
              extra='None|dict', filtered='None|dict', desc='None|str')
    def __init__(self, id_stream, shape, format, range, extra, #@ReservedAssignment
                 filtered=None, desc=None): #@ReservedAssignment
        
        self.id_stream = id_stream
        self.desc = desc
        self.extra = extra
        self.filtered = filtered
        
        self.streamels = np.zeros(shape=shape, dtype=streamel_dtype)
        self.kind = self.streamels['kind']
        self.lower = self.streamels['lower']
        self.upper = self.streamels['upper']
        
        # If the "format" is a string it is valid for all of them
        if isinstance(format, str):
            expect_one_of(format, ValueFormats.valid)
            self.kind[:] = format
            # at this point the range must be unique
            check_valid_bounds(range)
            self.lower[:] = range[0]
            self.upper[:] = range[1]
        else:
            # If the format is not a string, then it must be a list
            if not isinstance(format, list):
                raise ValueError('Expected list, got %s' % format)
            # And it must have the same number of elements
            formats = np.array(format)
            if formats.shape != self.streamels.shape:
                msg = ('Expected format shape to be %s instead of %s.' % 
                       (formats.shape, self.streamels.shape)) 
                raise ValueError(msg)
            for i in xrange(formats.size):
                expect_one_of(formats[i], ValueFormats.valid)
                self.kind[i] = formats[i]
            # Also the range must be a list
            assert range is not None 
            range = np.array(range) #@ReservedAssignment
            # With the same number of elements
            if range.shape[:-1] != self.streamels.shape:
                msg = ('Expected range to have shape %r instead of %r' % 
                       (self.streamels.shape, range.shape))
                raise ValueError(msg)
            # Each element of range must be a list with two elements
            for i, bounds in enumerate(range):
                check_valid_bounds(bounds)
                self.lower[i] = bounds[0]
                self.upper[i] = bounds[1]
                
    
    def __eq__(self, other):
        return np.all(self.streamels == other.streamels) 
        
        
    def size(self):
        return self.streamels.size
    
    @staticmethod
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
                
            return StreamSpec(id_stream, shape, format, range,
                                    extra, filtered, desc)
        except:
            logger.info('Error while parsing the StreamSpec:\n%s' % spec)
            raise
        
    def to_yaml(self):
        arange = []
        for i in range(self.streamels.size):
            arange.append((self.lower[i], self.upper[i]))
            
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
#    
#    @contract(returns='int,>=0')
#    def number(self):
#        ''' Returns the number of sensels. '''
#        pass # XXX
#
#    @contract(returns='tuple')
#    def shape(self):
#        ''' Returns the shape of the sensels. '''
#        pass # XXX

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
        
        return StreamSpec(id_stream, shape, format, mrange, extra,
                          filtered=filtered, desc=desc)



def check_valid_bounds(bounds):
    if not isinstance(bounds, (list, np.ndarray)):
        msg = 'Expect list or array, got %s.' % bounds
        raise ValueError(msg) 
    expect_size(bounds, 2) 
    if not bounds[0] < bounds[1]:
        raise ValueError('Invalid bounds [%s,%s]' % 
                         (bounds[0], bounds[1]))


def expect_one_of(x, options):
    if not x in options:
        msg = ('Got %r, expected one of %s' % (x, options))
        raise ValueError(msg)


def expect_size(x, l):
    if not len(x) == l:
        msg = 'Expected len %s, got %s.' % (l, len(x))
        raise ValueError(msg)
