from .with_queue import WithQueue
from blocks import SimpleBlackBoxT
from contracts import contract
from contracts.utils import raise_wrapped



__all__ = [
    'LastNSamples',
    'LastNSamplesT',
]


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

    def _check_consistent(self):
        pass
    
    def put_noblock(self, value):
        self.last.append(value)
        try:
            self._check_consistent()
        except ValueError as e:
            raise_wrapped(ValueError, e, 
                          'Error while putting value',
                          value=value)
        
        
        if len(self.last) >= self.N:
            out = self.last[:self.N]
            self.append(out)
            self.last = self.last[1:]
        else:
            if self.send_partial:
                self.append(list(self.last))

    def end_input(self):
#         if self.send_partial:
#             while self.last:
#                 self.append(list(self.last))
#                 self.last.pop(0)

        WithQueue.end_input(self)


class LastNSamplesT(LastNSamples, SimpleBlackBoxT):
    def _check_consistent(self):
        from blocks.library.timed.checks import check_timed

        timestamps = []
        for x in self.last:
            check_timed(x)
            t, _ = x
            timestamps.append(t)
        
        for i in range(len(timestamps)-1):
            t0 = timestamps[i]
            t1  = timestamps[i+1]
            if not t0 < t1:
                msg = 'Invalid Series of timestamps: %s' % timestamps
                raise ValueError(msg)
                    
                
                
                
                
                
                
                
        
    