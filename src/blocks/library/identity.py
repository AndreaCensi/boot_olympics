from .with_queue import WithQueue


__all__ = ['Identity']

    
class Identity(WithQueue):

    def reset(self):
        pass

    def put_noblock(self, value):
        self.append(value)
        
