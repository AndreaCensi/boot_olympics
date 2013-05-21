from .base import (BOOT_OLYMPICS_SENSEL_RESOLUTION, check_valid_streamels,
    streamel_dtype, ValueFormats)
from bootstrapping_olympics import logger
from bootstrapping_olympics.utils import (assert_allequal_verbose,
    assert_allclose_verbose, display_some, display_some_extended)
from contracts import check, describe_type, describe_value, contract
from numbers import Number
import numpy as np
    
__all__ = ['StreamSpec', 'BootInvalidValue', 'streamels_all_of_kind']
 
# TODO: check how it is used
class BootInvalidValue(ValueError):
    pass


class StreamSpec(object):
    """ 
    
        The simplest way to describe this is by using the ``from_yaml`` 
        function, which can take structures like these: :: 
        
            observations: 
                shape: [180]
                format: C
                range: [0,1]
            commands:
                shape: [2]
                format: [C, C]
                range: [[-1,+1],[-1,+1]]
                names: ['linear velocity', 'angular velocity']

        or using directly ``streamels_from_spec``:
        
            spec = streamels_from_spec()
         
    """
        

    @contract(id_stream='None|str', streamels='streamel_array',
              extra='None|dict', filtered='None|dict', desc='None|str')
    def __init__(self, id_stream, streamels, extra={},
                 filtered=None, desc=None):

        self.id_stream = id_stream
        self.desc = desc
        self.extra = extra
        self.filtered = filtered

        self.streamels = streamels
        self.kind = self.streamels['kind']
        self.lower = self.streamels['lower']
        self.upper = self.streamels['upper']

        check_valid_streamels(streamels)

    def __repr__(self):
        return 'StreamSpec(shape=%s)' % str(self.streamels.shape)

    def __eq__(self, other):
        return np.all(self.streamels == other.streamels)

    @staticmethod
    def check_same_spec(spec1, spec2):
        s1 = spec1.get_streamels()
        s2 = spec2.get_streamels()
        assert_allequal_verbose(s1['kind'], s2['kind'])
        assert_allclose_verbose(s1['lower'], s2['lower'])
        assert_allclose_verbose(s1['upper'], s2['upper'])
        assert_allclose_verbose(s1['default'], s2['default'])

    def get_streamels(self):
        return self.streamels.copy()

    @contract(returns='array')
    def get_default_value(self):
        ''' 
            Returns a "default value" for this stream. 
            
            For commands streams, this has the semantics of being
            a "at rest" value. 
            
            For observations, this is an "example"
            value, used for visualization. 
        '''
        return self.streamels['default']

    @contract(returns='array')
    def get_random_value(self):
        ''' 
            Returns a "random value" for this stream. 
        
            This is distributed uniformly in the lower/upper bound range. 
            Note this is not intrinsic and is representation dependent.
        '''
        x = np.zeros(self.streamels.shape)

        streamelsf = self.streamels.flat
        for i in xrange(x.size):
            lower = streamelsf[i]['lower']
            upper = streamelsf[i]['upper']
            kind = streamelsf[i]['kind']
            if kind == ValueFormats.Continuous:
                val = np.random.uniform(lower, upper)
            elif kind == ValueFormats.Discrete:
                val = np.random.randint(lower, upper + 1)
            elif kind == ValueFormats.Invalid:
                val = np.NaN
            else:
                assert False
            x.flat[i] = val
        return x

    @contract(x='array')
    def check_valid_value(self, x):
        ''' 
            Checks if the value x is valid according to this spec
            and raises BootInvalidValue (derived from ValueError) if it is not.
         '''

        def bail(msg):
            msg += '\n  stream: %s' % self
            msg += '\n   value: %s' % describe_value(x)
            raise BootInvalidValue(msg)

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
        if not np.all(np.isnan(x[invalids])):  # XXX: slow
            bail('Not all invalids are set to NaN.')

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

        lv = self.lower[valid]  # XXX: is this only 1D?
        uv = self.upper[valid]
        out_of_range = np.logical_or(xv < lv, xv > uv)
        if np.any(out_of_range):
            # print 'bounds dtype', lv.dtype, uv.dtype
            msg = ('Found elements out of range:\n %s' % 
                   display_some_extended(xv, self.streamels[valid], out_of_range))
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
            check('list[>0](int,>0)', shape)  # XXX: slow
            format = s.pop('format')  # @ReservedAssignment
            range = s.pop('range')  # @ReservedAssignment
            extra = s.pop('extra', {})
            filtered = s.pop('filtered', None)
            default = s.pop('default', None)

            names = s.pop('names', None)  # TODO: do something useful with this
            if names:
                extra['names'] = names

            if s.keys():
                logger.warning('While reading\n%s\nextra keys detected: %s' % 
                               ((spec), s.keys()))

            streamels = streamels_from_spec(shape, format, range, default)

            return StreamSpec(id_stream, streamels, extra, filtered, desc)
        except:
            logger.error('Error while parsing the StreamSpec:\n%s' % spec)
            raise

    @contract(returns='dict')
    def to_yaml(self):
        arange = []

        data = {
            'id': self.id_stream,
            'desc': self.desc,
            'extra': self.extra,
            'filtered': self.filtered,
            'shape': list(self.streamels.shape)
        }

        if all_same_spec(self.streamels):
            # print("OK, we can optimize for %s" % self.streamels)
            # We can optimize the representation
            s = self.streamels.flat[0]
            data['format'] = s['kind'].item()
            data['default'] = s['default'].item()
            data['range'] = [s['lower'].item(), s['upper'].item()]

        else:
            # print("Sorry, we can't optimize for %s" % self.streamels)
            if self.streamels.ndim == 1:
                for i in range(self.streamels.size):
                    arange.append(get_streamel_range(self.streamels[i]))
            elif self.streamels.ndim == 2:
                for i in range(self.streamels.shape[0]):
                    row = []
                    for j in range(self.streamels.shape[1]):
                        row.append(get_streamel_range(self.streamels[i, j]))
                    arange.append(row)
            else:
                assert False

            data['format'] = self.streamels['kind'].tolist()
            data['default'] = self.streamels['default'].tolist()
            data['range'] = arange

        return data

    def __str__(self):
        return 'StreamSpec(%s)' % str(self.streamels.shape)

    @staticmethod
    def join(obs1, obs2):
        ''' Returns the spec obtained by concatenating two of them. '''
        assert isinstance(obs1, StreamSpec)
        assert isinstance(obs2, StreamSpec)
        if len(obs1.streamels.shape) != 1:
            raise Exception('join() not implemented for 2D signals')
        if len(obs2.streamels.shape) != 1:
            raise Exception('join() not implemented for 2D signals')
        joint = np.hstack((obs1.streamels, obs2.streamels))
        assert len(joint.shape) == 1

        # XXX: this can be done better
        # names TODO
        id_stream = '%s-%s' % (obs1.id_stream, obs2.id_stream)
        shape = list(joint.shape)
        format = joint['kind'].tolist()  # @ReservedAssignment
        mrange = np.zeros(shape=joint.shape + (2,))  # XXX: use int?
        mrange[..., 0] = joint['lower']
        mrange[..., 1] = joint['upper']
        mrange = mrange.tolist()
        extra = {}
        filtered = dict(filter='join',
                        original=[obs1.to_yaml(), obs2.to_yaml()])
        desc = 'Join of %s and %s' % (obs1.id_stream, obs2.id_stream)

        default = joint['default'].tolist()  # XXX --- not tested
        streamels = streamels_from_spec(shape=shape, format=format,
                                        range=mrange, default=default)
        return StreamSpec(id_stream=id_stream,
                          streamels=streamels, extra=extra,
                          filtered=filtered, desc=desc)

    def get_id_stream(self):
        return self.id_stream


def streamels_all_of_kind(streamels, kind):
    """ Returns True if all streamels are of the same given kind ('C','D') """
    return np.all(streamels['kind'] == kind)


def all_same_spec(streamels):
    ''' Checks that all streamels have the same attributes. '''
    for s in ['upper', 'lower', 'kind', 'default']:
        if not only_one_value(streamels[s]):
            # print('Cannot optimize because of %s' % s)
            return False
    return True


def only_one_value(array):
    """ Checks that the array has only one value. """
    un = np.unique(array)
    if len(un) == 1:
        return True
    else:
        # print('Different values: %s' % un)
        return False


def check_valid_bounds(bounds):
    if not isinstance(bounds, (list, np.ndarray)):
        msg = 'Expect list or array, got %s.' % bounds
        raise ValueError(msg)
    expect_size(bounds, 2)
    if not bounds[0] < bounds[1]:
        msg = ('Invalid bounds lower: %s upper: %s (lower>=upper)' % 
                (bounds[0], bounds[1]))
        raise ValueError(msg)


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
        # do not use nan, otherwise streamels!=streamels
        streamel['lower'] = np.inf
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
def streamels_from_spec(shape, format, range, default):  # @ReservedAssignment
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
            msg = ('Expected list for "format", got %s.'
                   % describe_value(format))
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

        # also 'default' must 

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

    # and also the default value must be a number
    if default is None:
        # Use half of the range
        half = (streamels['upper'] + streamels['lower']) / 2.0
        streamels['default'][:] = half  # XXX
        # round down, discretize if discrete
        isdiscrete = kind == ValueFormats.Discrete
        rounded = np.floor(half)
        streamels['default'][isdiscrete] = rounded[isdiscrete]
    elif isinstance(default, Number):
        # 
        streamels['default'].flat[:] = default
    elif isinstance(default, list):  # explicit list
        defaults = np.array(default, dtype=BOOT_OLYMPICS_SENSEL_RESOLUTION)
        if defaults.shape != streamels.shape:
            msg = ('Expected defaults shape to be %s instead of %s.' % 
                   (defaults.shape, streamels.shape))
            raise ValueError(msg)
        if not (np.issubdtype(defaults.dtype, int) or
                np.issubdtype(defaults.dtype, float)):
            msg = ('Expect an array of numbers, not %s.'
                   % describe_value(defaults))
            raise ValueError(msg)
        streamels['default'].flat[:] = defaults.flat
    else:
        msg = 'Could not interpret default value %s.' % describe_value(default)
        raise ValueError(msg)
    return streamels



