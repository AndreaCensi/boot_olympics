from blocks import SimpleBlackBoxTN
from blocks.library import WithQueue
from contracts import contract

__all__ = [
    'SyncRep',
]


class SyncRep(WithQueue, SimpleBlackBoxTN):
    """ 
        Replicate some signals to match the master. 
    
        There is a "master" signal.
            
        Each time the master arrives, t
    
    """
    
    @contract(master='str', slaves='seq(str)')
    def __init__(self, master, slaves):
        self.master = master
        self.slaves = list(slaves)
        
    def reset(self):
        WithQueue.reset()
        
    def put_noblock(self, value):
        pass