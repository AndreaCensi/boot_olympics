from blocks import SimpleBlackBox
 
__all__ = ['WithQueue', 'Identity']
 
class WithQueue(SimpleBlackBox):
    def __init__(self):
        self.queue = []
          
    def get(self, block=True, timeout=None):  # @UnusedVariable
        if not self.queue:
            raise SimpleBlackBox.NotReady()
        return self.queue.pop(0)
    
    def append(self, value):
        self.queue.append(value)
        if len(self.queue) > 100:
            print('%s: Warning, too much growth? %s' % (type(self), len(self.queue)))
    
class Identity(WithQueue):
    def put(self, x):
        self.queue.append(x)
     
def bb_pump(a, b):
    while True:
        try:
            x = a.get(block=False)
        except SimpleBlackBox.NotReady:
            break
        b.put(x)
#         print('pump %s -> %s: %s ' % (type(a).__name__,
#                                       type(b).__name__, x))
#         
