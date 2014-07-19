from blocks.library.simple.with_queue import WithQueue
from blocks.library.simple.instantaneous import Instantaneous


__all__ = ['Identity']

    
class Identity(Instantaneous):

    def transform_value(self, value):
        return value
        
    def __str__(self):
        return 'Identity()'


