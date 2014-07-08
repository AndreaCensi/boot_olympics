import warnings

from contracts import contract, describe_type, describe_value
from contracts.utils import deprecated

from blocks import SimpleBlackBox, Source, Sink
from blocks.exceptions import NeedInput
from blocks.utils import check_reset

from .exceptions import Full, NotReady, Finished
from .pumps import bb_pump
from blocks.pumps import bb_pump_block


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
def series_two(a, b, name1=None, name2=None):
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
    def __init__(self, a, b, name1=None, name2=None):
        self.a = a
        self.b = b
        self.log_add_child(name1, a)
        self.log_add_child(name2, b)

    def __str__(self):
        return 'SourceBBSeries(%s,%s)' % (self.a, self.b)

    @contract(names='list[>=2](str)')
    def set_names(self, names):
        if len(names) == 2:
            self.log_add_child(names[0], self.a)
            self.log_add_child(names[1], self.b)
        else:
            self.log_add_child(names[0], self.a)
            self.b.set_names(names[1:])

    def reset(self):
        self.a.reset()
        self.b.reset()
        self.reset_once = True

    @contract(block='bool', timeout='None|>=0', returns='*')
    def get(self, block=True, timeout=None):
        check_reset(self, 'reset_once')

        # XXX: not sure any of this is correct
        # self.info('%s trying to get' % id(self))
        try:
            return self.b.get(block=block, timeout=timeout)
        except NeedInput:
            try:
                # XXX: all of this is not really tested or thought out
                r = self.a.get(block=block, timeout=timeout)
                self.b.put(r, block=block, timeout=timeout)
                return self.get(block=block, timeout=timeout)
            except NeedInput:
                # XXX does it make sense to call NeedInput on Source?
                raise NeedInput()
            except Finished:
                # self.info('a finished: setting b.end_input()')
                self.b.end_input()
                return self.get(block=block, timeout=timeout)
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


    @contract(names='list[>=2](str)')
    def set_names(self, names):
        if len(names) == 2:
            self.log_add_child(names[0], self.a)
            self.log_add_child(names[1], self.b)
        else:
            self.log_add_child(names[0], self.a)
            self.b.set_names(names[1:])

    def reset(self):
        self.a.reset()
        self.b.reset()
        self.reset_once = True

    @contract(block='bool', timeout='None|>=0', returns='None')
    def put(self, value, block=True, timeout=None):
        check_reset(self, 'reset_once')
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

    def __str__(self):
        return 'BBBBSeries(%s, %s)' % (self.a, self.b)

    @contract(names='list[>=2](str)')
    def set_names(self, names):
        if len(names) == 2:
            self.log_add_child(names[0], self.a)
            self.log_add_child(names[1], self.b)
        else:
            self.log_add_child(names[0], self.a)
            self.log_add_child('s:' + '+'.join(names[1:]), self.b)
            self.b.set_names(names[1:])

    def reset(self):
        self.reset_once = True

        self.status_a_finished = False
        self.status_a_need_input = False
        self.status_b_finished = False
        self.status_b_need_input = False

        self.a.reset()
        self.b.reset()

    def end_input(self):
#         self.info('end_input()')
        check_reset(self, 'reset_once')
        self.a.end_input()
        self._pump()

    def put(self, value, block=True, timeout=None):
        check_reset(self, 'reset_once')
#         self.info('put(): %s' % str(value))

        assert not self.status_a_finished

        # XXX: not sure this is corect
        self.a.put(value, block=block, timeout=timeout)
        self.status_a_need_input = False
        self.status_a_finished = False


        self._pump()
#         try:
#             bb_pump(self.a, self.b)
#         except Finished:
#             self.b.end_input()

    def _pump(self):        
#         self.info('_pump()')
        
        num = 0
        while True:
            try:
                x = self.a.get(block=True)
            except NotReady:
                assert False
            except NeedInput:
                self.status_a_need_input = True
                break
            except Finished:
                self.status_a_finished = True
                self.b.end_input()
                break

            self.b.put(x, block=True)
            self.status_b_need_input = True
            num += 1

    def log_get_short_status(self):
        if not 'reset_once' in self.__dict__:
            return 'no reset'
        return '%s:%s%s; %s:%s%s' % (self.log_child_name(self.a),
                                         ' ni' if self.status_a_need_input else '',
                                       ' f' if self.status_a_finished else '',
                                       self.log_child_name(self.b),
                                        ' ni' if self.status_b_need_input else '',
                                       ' f' if self.status_b_finished else '')
#
#         try:
#             bb_pump(self.a, self.b)
# #             bb_pump_block(self.a, self.b)
#         except Finished:
#             self.b.end_input()
#         except NotReady:
#             assert False
#         except NeedInput:
#             self.info('bb_pump_block: NeedInput')
#             raise

    def get(self, block=True, timeout=None):
        check_reset(self, 'reset_once')
#         self.info('trying to get() from %s' % self.log_child_name(self.b))
        try:
            return self.b.get(block=block, timeout=timeout)
        except NeedInput:
            assert block == True
            self.status_b_need_input = True
            if self.status_a_need_input:
                raise NeedInput()
            else:
                self._pump()
                return self.get(block=block, timeout=timeout)
        except NotReady:
            assert block == False
            if block:
                msg = 'Bug, cannot get NotReady if block is true.'
                msg += '\n %s' % describe_value(self.b)
                raise Exception(msg)
            # self.info('b is not ready')
            raise
        except Finished:
            self.status_b_finished = True
            # self.info('b is finished')
            raise
