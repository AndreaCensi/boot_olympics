from blocks import (Finished, NeedInput, NotReady, SimpleBlackBox, 
    SimpleBlackBoxT, SimpleBlackBoxTN)
from blocks.library import   WithQueue
from blocks.library.timed.checks import check_timed_named

__all__ = [
    'loopit',
    'Loopit',
    'LoopitT',
    'LoopitTN',
]


def loopit(M,  *args, **kwargs):
    if isinstance(M, SimpleBlackBoxTN):
        return LoopitTN(M,  *args,**kwargs)
    if isinstance(M, SimpleBlackBoxT):
        return LoopitT(M, *args, **kwargs)
    if isinstance(M, SimpleBlackBox):
        return Loopit(M,  *args,**kwargs)
    
class Loopit(WithQueue):
    """ Note: this might create an infinite loop..."""
    def __init__(self, B, outsignal, reinput_as):
        self.a = B
        self.outsignal = outsignal
        self.reinput_as = reinput_as
        
    def reset(self):
        WithQueue.reset(self)
        self.a.reset()
        
    def put_noblock(self, value):
        check_timed_named(value)
        # We put it to the internal box
        self.a.put(value, block=True)
        self._pump() 
        
    def end_input(self):
        self.a.end_input()
        self._pump()
        
    def _pump(self):
        num = 0
        while True:
            try:
                x = self.a.get(block=True)
            except NotReady:
                assert False
            except NeedInput:
                break
            except Finished:
                WithQueue.end_input(self)
                break
            self.append(x)
            num += 1
            
            check_timed_named(x)
            t, (signal, value) = x
            if signal == self.outsignal:
                self.put((t, (self.reinput_as, value))) 
    

class LoopitT(Loopit, SimpleBlackBoxT):
    pass

class LoopitTN(LoopitT, SimpleBlackBoxTN):
    pass