from blocks import SimpleBlackBox
 
__all__ = ['WithQueue']
 
class WithQueue(SimpleBlackBox):
    """ 
        A black box that is implemented by doing something every time
        the element is put() inside.
    
        Implement put() and use append() to output stuff.
    """
        
    def __init__(self):
        self.queue = []
          
    def get(self, block=True, timeout=None):  # @UnusedVariable
        if not self.queue:
            raise SimpleBlackBox.NotReady()
        return self.queue.pop(0)
    
    def append(self, value):
        """ Appends to the internal queue """
        self.queue.append(value)
        if len(self.queue) > 100:
            print('%s: Warning, too much growth? %s' 
                  % (type(self), len(self.queue)))
        
     
def bb_pump(a, b):
    while True:
        try:
            x = a.get(block=False)
        except SimpleBlackBox.NotReady:
            break
        b.put(x)
