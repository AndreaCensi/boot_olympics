from abc import abstractmethod

from blocks import NotReady, Finished, SimpleBlackBox
from blocks.utils import check_reset
from blocks.exceptions import NeedInput


__all__ = ['WithQueue']

class WithQueue(SimpleBlackBox):
    """ 
        A black box that is implemented by doing something every time
        the element is put() inside.
    
        Implement put_noblock() and use self.append() to output stuff.
    """

    @abstractmethod
    def put_noblock(self, value):
        pass

    def __init__(self):
        pass
    
    def __repr__(self):
        return '%s()' % type(self).__name__ 

    def reset(self):
        self._queue = []
        self._finished = False

    def get(self, block=True, timeout=None):  # @UnusedVariable
        check_reset(self, '_queue')
        if not self._queue:
            if self._finished:
                raise Finished()
            else:
                if block:
                    raise NeedInput()
                else:
                    raise NotReady()
        res = self._queue.pop(0)
        return res

    def append(self, value):
        """ Appends to the internal queue """
        check_reset(self, '_queue')
        self._queue.append(value)

    def end_input(self):
        self._finished = True

    def put(self, value, block=True, timeout=None):  # @UnusedVariable
        self.put_noblock(value)
    
