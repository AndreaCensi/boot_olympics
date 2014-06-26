from contracts import contract, describe_value

from .exceptions import NotReady, Finished
from .pumps import bb_pump
from blocks.interface import SimpleBlackBox, Source


__all__ = ['series']


@contract(a='isinstance(SimpleBlackBox)|isinstance(Source)',
          b='isinstance(SimpleBlackBox)')
def series(a, b, name1='a', name2='b'):
    if isinstance(a, SimpleBlackBox):
        return SimpleBlackBoxSeries(a, b, name1, name2)
    if isinstance(a, Source):
        return SourceBBSeries(a, b, name1, name2)
    assert(False)


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
        self.info('%s trying to get' % id(self))
        try:
            return self.b.get(block=block, timeout=timeout)
        except NotReady:
            self.info('b not ready (b: %s)' % describe_value(self.b))
            try:
                r = self.a.get(block=block, timeout=timeout)
                self.info('Read from a, putting in b')
                self.b.put(r, block=block, timeout=timeout)
                return self.get(block=block, timeout=timeout)
            except NotReady:
                self.info('b not ready: raising NotReady')
                raise
            except Finished:
                self.info('a finished: setting b.end_input()')
                self.b.end_input()
                return self.get(block=block, timeout=timeout)
        except Finished:
            self.info('b finished: we are finished')
            raise

  
class SimpleBlackBoxSeries(SimpleBlackBox):
    """ Implements series between two SimpleBlackBoxes """

    @contract(a=SimpleBlackBox, b=SimpleBlackBox)
    def __init__(self, a, b, name1='a', name2='b'):
        self.a = a
        self.b = b
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

#     def get_typsy_type(self):
#         ta = self.a.get_typsy_type()
#         tb = self.b.get_typsy_type()
#         return BlackBox(ta.i, tb.o, ta.t)

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
#         self.a.put(value, block=block, timeout=None)
#         while True:
#             try:
#                 r = self.a.get(block=False, timeout=0)
#             except NotReady:
#                 break
#             self.b.write(r)

    def get(self, block=False, timeout=None):
        self.info('trying to get from b')
        try:
            return self.b.get(block=block, timeout=timeout)
        except NotReady:
            self.info('b is not ready')
            raise
        except Finished:
            self.info('b is finished')
            raise

#         if block:
#             return bb_get_block_poll_sleep(self, timeout=timeout,
#                                            sleep=self.sleep)
#         else:
#             return self._get_notblock()
#
