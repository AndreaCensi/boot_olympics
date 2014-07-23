from .exceptions import Finished, Full, NeedInput, NotReady
from .interface import (SimpleBlackBox, SimpleBlackBoxT, SimpleBlackBoxTN, Sink, 
    Source)
from .pumps import bb_pump
from .utils import check_reset
from contracts import contract, describe_type, describe_value
from contracts.utils import deprecated, indent, raise_wrapped
import warnings


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
    if isinstance(a, SimpleBlackBoxTN) and isinstance(b, SimpleBlackBox):
        return BBBBSeriesTN(a, b, name1, name2)
    
    if isinstance(a, SimpleBlackBoxT) and isinstance(b, SimpleBlackBox):
        return BBBBSeriesT(a, b, name1, name2)
    
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

    def __repr__(self):
        s = _format_multiline(type(self).__name__, 
                              self.log_child_name(self.a), self.a, 
                              self.log_child_name(self.b), self.b)
        return s

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
        check_reset(self, 'reset_once')
        self.a.end_input()
        self._pump()

    def put(self, value, block=True, timeout=None):
        check_reset(self, 'reset_once')

        assert not self.status_a_finished

        
        #msg = 'putting into %r value = %s.' % (self.log_child_name(self.a), describe_value(value))
        #self.debug(msg)
        
        # XXX: not sure this is correct (block behavior)
        try:
            self.a.put(value, block=block, timeout=timeout)
        # TODO: other exceptions
        except NotReady:
            raise
        except BaseException:
            msg = 'Error while trying to put() into %r.' % self.log_child_name(self.a)
            msg += '\n a = %s' % describe_value(self.a)
            msg += '\n of type %s' % describe_type(self.a)
            msg += '\n value = %s' % describe_value(value)
            self.error(msg)
            raise
            
        self.status_a_need_input = False
        self.status_a_finished = False

        self._pump()

    def _pump(self):        
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
            except BaseException:
                msg = 'Error while trying to get() from %r.' % self.log_child_name(self.a)
                msg += '\n a = %s' % describe_value(self.a)
                msg += '\n of type %s' % describe_type(self.a)
                self.error(msg)
                raise

            try:
                self.b.put(x, block=True)
            except BaseException:
                msg = 'Error while trying to put() into %r.' % self.log_child_name(self.b)
                msg += '\n a = %s' % describe_value(self.b)
                msg += '\n of type %s' % describe_type(self.b)
                msg += '\n value = %s' % describe_value(x)

                self.error(msg)
                raise

            self.status_b_need_input = True
            num += 1

#     def log_get_short_status(self):
#         if not 'reset_once' in self.__dict__:
#             return 'no reset'
#         return '%s:%s%s; %s:%s%s' % (self.log_child_name(self.a),
#                                          ' ni' if self.status_a_need_input else '',
#                                        ' f' if self.status_a_finished else '',
#                                        self.log_child_name(self.b),
#                                         ' ni' if self.status_b_need_input else '',
#                                        ' f' if self.status_b_finished else '')

    def get(self, block=True, timeout=None):
        check_reset(self, 'reset_once')
        try:
            return self.b.get(block=block, timeout=timeout)
        except NeedInput as e:
            assert block == True
            self.status_b_need_input = True
            if self.status_a_need_input:
                raise_wrapped(NeedInput, e,
                              'Both need input',
                              a=self.a, b=self.b)
            else:
                self._pump()
                return self.get(block=block, timeout=timeout)
        except NotReady:
            assert block == False
            if block:
                msg = 'Bug, cannot get NotReady if block is true.'
                msg += '\n %s' % describe_value(self.b)
                raise Exception(msg)
            raise
        except Finished:
            self.status_b_finished = True
            raise
        
class BBBBSeriesT(BBBBSeries, SimpleBlackBoxT):
    pass
class BBBBSeriesTN(BBBBSeries, SimpleBlackBoxTN):
    pass


def _format_multiline(s, name_a, a, name_b, b):
    res = s 
    maxlen = max(len(name_a), len(name_b))
    name_a = ' '*(maxlen-len(name_a)) + name_a
    name_b = ' '*(maxlen-len(name_b)) + name_b
    res += '\n' + indent(a.__repr__(), ' |', '%s|' % name_a)
    res += '\n' + indent(b.__repr__(), ' |', '%s|' % name_b)
    return res
    
    
    
    


    
