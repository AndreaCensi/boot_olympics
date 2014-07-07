from .with_queue import WithQueue


__all__ = ['Identity']

    
class Identity(WithQueue):

    def put_noblock(self, value):
        self.append(value)
        

    def __str__(self):
        return 'Identity()'
