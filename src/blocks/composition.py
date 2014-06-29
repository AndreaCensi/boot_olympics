import warnings

from contracts import contract, describe_type
from contracts.utils import deprecated

from blocks import SimpleBlackBox, Source, Sink

from .exceptions import Full, NotReady, Finished
from .pumps import bb_pump


__all__ = [
    'series',
    'series_multi',
]


@deprecated
def series_multi(*args):
    return series(*args)


def series(*args):
    if len(args) < 2:
        msg = 'Expected at least 2 args, got %d.' % len(args)
        msg += '\n %s' % str(args)
        raise ValueError(msg)
    elif len(args) == 2:
        return series_two(*args)
    elif len(args) > 2:
        return series_two(args[0], series(*args[1:]))
    else:
        assert False


@contract(a='isinstance(SimpleBlackBox)|isinstance(Source)',
          b='isinstance(SimpleBlackBox)|isinstance(Sink)')
def series_two(a, b, name1='a', name2='b'):
    if isinstance(a, SimpleBlackBox) and isinstance(b, SimpleBlackBox):
        return BBBBSeries(a, b, name1, name2)
    if isinstance(a, Source) and isinstance(b, SimpleBlackBox):
        return SourceBBSeries(a, b, name1, name2)
    if isinstance(a, SimpleBlackBox) and isinstance(b, Sink):
        return BBSinkSeries(a, b, name1, name2)

    msg = 'Cannot find proper interconnection'
    msg += '\n %10s: %s' % (name1, describe_type(a))
    msg += '\n %10s: %s' % (name2, describe_type(b))
    raise ValueError(msg)



class SourceBBSeries(Source):
    """ Implements series between a Source and a SimpleBlackBox """

    @contract(a=Source, b=SimpleBlackBox)
    def __init__(self, a, b, name1='a', name2='b'):
        self.a = a
        self.b = b
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

    @contract(block='bool', timeout='None|>=0', returns='*')
    def get(self, block=True, timeout=None):
        # XXX: not sure any of this is correct
        # self.info('%s trying to get' % id(self))
        try:
            return self.b.get(block=block, timeout=timeout)
        except NotReady:
            # self.info('b not ready (b: %s)' % describe_value(self.b))
            try:
                r = self.a.get(block=block, timeout=timeout)
                # self.info('Read from a, putting in b')
                self.b.put(r, block=block, timeout=timeout)
                return self.get(block=block, timeout=timeout)
            except NotReady:
                # self.info('b not ready: raising NotReady')
                raise
            except Finished:
                # self.info('a finished: setting b.end_input()')
                self.b.end_input()
                return self.get(block=block, timeout=timeout)
        except Finished:
            # self.info('b finished: we are finished')
            raise


class BBSinkSeries(Sink):
    """ Implements series between a Source and a SimpleBlackBox """

    @contract(a=SimpleBlackBox, b=Sink)
    def __init__(self, a, b, name1='bb', name2='sink'):
        self.a = a
        self.b = b
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

    @contract(block='bool', timeout='None|>=0', returns='None')
    def put(self, value, block=False, timeout=None):
        try:
            self.a.put(value, block=block, timeout=timeout)
        except Full:
            warnings.warn('to implement')
            raise
        self._pump()

    def _pump(self):
        try:
            bb_pump(self.a, self.b)
        except Finished:
            self.b.end_input()

    def end_input(self):
        self.a.end_input()
        self._pump()
  


class BBBBSeries(SimpleBlackBox):
    """ Implements series between two SimpleBlackBoxes """

    @contract(a=SimpleBlackBox, b=SimpleBlackBox)
    def __init__(self, a, b, name1='bb1', name2='bb2'):
        self.a = a
        self.b = b
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

    def end_input(self):
        # self.info('Signaled end of input. Telling a (%s)' % type(self.a))
        self.a.end_input()
        try:
            bb_pump(self.a, self.b)
        except Finished:
            self.b.end_input()

    def put(self, value, block=False, timeout=None):
        # XXX: not sure this is corect
        self.a.put(value, block=block, timeout=timeout)
        try:
            bb_pump(self.a, self.b)
        except Finished:
            self.b.end_input()

    def get(self, block=False, timeout=None):
        # self.info('trying to get from b')
        try:
            return self.b.get(block=block, timeout=timeout)
        except NotReady:
            # self.info('b is not ready')
            raise
        except Finished:
            # self.info('b is finished')
            raise
