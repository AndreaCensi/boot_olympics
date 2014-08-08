from blocks import SimpleBlackBoxTN
from blocks.library import WithQueue
from contracts import contract
from blocks.library.timed import check_timed_named


__all__ = [
    'SyncRep',
]


class SyncRep(WithQueue, SimpleBlackBoxTN):
    """ 
        Replicate some signals to match the master. 
    
        There is a "master" signal.
            
        Each time the master arrives, it is output
        and then we output the last values of all other signals
        with the master's timestamp.
    
    """
    
    @contract(master='str')
    def __init__(self, master):
        self.master = master
        
    def reset(self):
        WithQueue.reset(self)
        self.last = {}
        
    def put_noblock(self, value):
        check_timed_named(value)
        t, (n, s) = value
        if n == self.master:
            # pass it through
            self.append(value)
            for slave, slave_value in self.last.items():
                self.append((t, (slave, slave_value)))
        else:
            self.last[n] = s
            
            