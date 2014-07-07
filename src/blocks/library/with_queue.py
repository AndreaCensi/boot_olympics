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

    def reset(self):
        self._queue = []
        self._finished = False

    def get(self, block=True, timeout=None):  # @UnusedVariable
        check_reset(self, '_queue')
        self.info('trying to get(): qlen: %d finished: %d' % (len(self._queue), self._finished))
        if not self._queue:
            if self._finished:
                self.info('finished')
                raise Finished()
            else:
                if block:
                    self.info('NeedInput')
                    raise NeedInput()
                else:
                    self.info('not ready')
                    raise NotReady()
        res = self._queue.pop(0)
        self.info('returned %s' % str(res))
        return res

    def append(self, value):
        """ Appends to the internal queue """
        check_reset(self, '_queue')
        self._queue.append(value)

    def end_input(self):
        print('end_input() called for %s' % self)
        self._finished = True

    def put(self, value, block=False, timeout=None):  # @UnusedVariable
        # XXX
        self.info('put(%s)' % str(value))
        self.put_noblock(value)
    
