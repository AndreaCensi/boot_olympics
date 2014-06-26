from blocks.exceptions import NotReady, Finished

from .simple_black_box import SimpleBlackBox


__all__ = ['WithQueue']

class WithQueue(SimpleBlackBox):
    """ 
        A black box that is implemented by doing something every time
        the element is put() inside.
    
        Implement put() and use append() to output stuff.
    """

    def __init__(self):
        self._queue = []
        self._finished = False

    def get(self, block=True, timeout=None):  # @UnusedVariable
        self.info('trying to get(): qlen: %d finished: %d' % (len(self._queue), self._finished))
        if not self._queue:
            if self._finished:
                self.info('finished')
                raise Finished()
            else:
                self.info('not ready')
                raise NotReady()
        return self._queue.pop(0)

    def append(self, value):
        """ Appends to the internal queue """
        self._queue.append(value)

    def end_input(self):
        # print('end_input() called for %s' % self)
        self._finished = True


#         if len(self._queue) > 100:
#             print('%s: Warning, too much growth? %s'
#                   % (type(self), len(self._queue)))

