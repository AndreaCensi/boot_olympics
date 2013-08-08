from blocks.utils import WithQueue

    
class Identity(WithQueue):
    def put(self, x):
        self.queue.append(x)
        