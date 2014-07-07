from contracts import contract

from .with_queue import WithQueue


__all__ = ['LastNSamples']


class LastNSamples(WithQueue):
    """ 
        Collects N samples. Output is a list of length N. 
        
        If send_partial is True also less than N sets are sent at the beginning.
    
    """

    @contract(N='int')
    def __init__(self, N, send_partial=False):
        WithQueue.__init__(self)
        self.N = N
        self.send_partial = send_partial

    def reset(self):
        WithQueue.reset(self)
        self.last = []

    def put_noblock(self, value):
        self.last.append(value)
        if len(self.last) >= self.N:
            out = self.last[:self.N]
            self.append(out)
            self.last = self.last[1:]
        else:
            if self.send_partial:
                self.append(list(self.last))

    def end_input(self):
        if self.send_partial:
            while self.last:
                self.append(list(self.last))
                self.last.pop(0)

        WithQueue.end_input(self)

