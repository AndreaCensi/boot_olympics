from blocks import (Finished, NeedInput, NotReady, SimpleBlackBox, 
    SimpleBlackBoxT, SimpleBlackBoxTN)
from blocks.library import WithQueue
from contracts import contract, describe_type, describe_value
from blocks.library.timed.checks import check_timed_named, check_timed


__all__ = [ 
    'WrapTimedNamed',
    'WrapTMfromT',
    'WrapT',
]


class WrapTimedNamed(WithQueue, SimpleBlackBoxTN):
    """ 
        Converts a block that does not use time or signal name
        to one that uses time and signal. 
        
         from  BB(B;A)  
        to    BB((time, (signal, B)), (time, (signal, A)))
    """

    @contract(inside=SimpleBlackBox)
    def __init__(self, inside):
        WithQueue.__init__(self)
        self.inside = inside
        self.last_t = None
        self.last_name = None
        self.log_add_child(None, inside)

    def __repr__(self):
        return 'WrapTimedNamed(%r)' % self.inside 
    
    def end_input(self):
        self.inside.end_input()
        if self.last_t is not None:
            self._pump()

    def reset(self):
        WithQueue.reset(self)
        self.inside.reset()

    def put_noblock(self, value):
        check_timed_named(value, self)
        self.last_t, (self.last_name, ob) = value
        self.inside.put(ob)
        self._pump()

    def _pump(self):
        while True:
            try:
                ob2 = self.inside.get(block=True)
                try: 
                    check_timed_named(ob2)
                except ValueError:
                    pass
                else:
                    msg = ('I did not my input to be already timednamed, '
                           'but only timed:\n %s' % describe_value(ob2))
                    self.error(msg)
                    raise ValueError(msg)

                try: 
                    check_timed(ob2)
                except ValueError:
                    pass
                else:
                    msg = ('I did not my input to be already timed. '
                           '\n %s' % describe_value(ob2))
                    self.error(msg)
                    raise ValueError(msg)



                value2 = self.last_t, (self.last_name, ob2)
                self.append(value2)
            except NotReady:
                assert False  # block == True
            except NeedInput:
                break
            except Finished:
                self._finished = True 
                break

class WrapTMfromT(WithQueue, SimpleBlackBoxTN):
    """ 
        from  BB((time, B);(time, A)) 
        to    BB((time, (name, B));(time, (name, A)))
    """
    
    @contract(inside=SimpleBlackBoxT, inside_name='None|str')
    def __init__(self, inside, inside_name=None):
        WithQueue.__init__(self)
        self.inside = inside
        self.last_name = None
        self.log_add_child(inside_name, inside)
    
    def __repr__(self):
        return 'WrapTMfromT(%r)' % self.inside

    def reset(self):
        WithQueue.reset(self)
        self.inside.reset()

    def end_input(self):
        self.inside.end_input()
        if self.last_name is not None:
            self._pump()

    def put_noblock(self, value):
        check_timed_named(value, self)
        t, (self.last_name, ob) = value

        try: 
            x = (t, ob)
            self.inside.put(x, block=True)
        except NotReady:
            assert False
        except BaseException:
            msg = 'Cannot put() into %r.' % self.log_child_name(self.inside)
            msg += '\n inside = %s' % describe_value(self.inside)
            msg += '\n of type %s' % describe_type(self.inside)
            msg += '\n value = %s' % describe_value(x)
            self.error(msg)
            raise

        
        self._pump()

    def _pump(self):
        while True:
            try:
                x = self.inside.get(block=True)
                check_timed(x,self)
                t, ob = x
                try: 
                    check_timed_named(x)
                except ValueError:
                    pass
                else:
                    msg = ('I did not exoect the input of self.inside to be already timednamed, '
                           'but only timed:\n %s' % describe_value(x))
                    self.error(msg)
                    raise ValueError(msg)
                
                value2 = t, (self.last_name, ob)
                self.append(value2)
            except NotReady:
                break
            except Finished:
                self._finished = True
                break
            except NeedInput:
                break


class WrapT(WithQueue, SimpleBlackBoxT):
    """ 
        from  BB(B;A)  
        to    BB((time, B), (time, A))
    """
    
    @contract(inside=SimpleBlackBox, inside_name='None|str')
    def __init__(self, inside, inside_name=None):
        WithQueue.__init__(self)
        self.inside = inside
        self.last_time = None
        self.log_add_child(inside_name, inside)

    def __repr__(self):
        return 'WrapT(%r)' % self.inside

    def reset(self):
        WithQueue.reset(self)
        self.inside.reset()

    def end_input(self):
        self.inside.end_input()
        if self.last_name is not None:
            self._pump()

    def put_noblock(self, value):
        check_timed(value, self)
        t, ob = value
        self.last_time = t
        self.inside.put(ob, block=True)
        self._pump()

    def _pump(self):
        while True:
            try:
                x = self.inside.get(block=True)
                try: 
                    check_timed(x)
                except ValueError:
                    pass
                else:
                    msg = ('I did not exoect the input of self.inside to be already timed. '
                           '\n x = %s' % describe_value(x))
                    self.error(msg)
                    raise ValueError(msg)
                
                value2 = self.last_time, x
                self.append(value2)
            except NotReady:
                break
            except Finished:  # XXX
                self._finished = True
            except NeedInput:
                # XXX
                break
        