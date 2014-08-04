from .instantaneous import Instantaneous

__all__ = [
    'Identity',
]

class Identity(Instantaneous):

    def transform_value(self, value):
        return value
        
    def __str__(self):
        return 'Identity()'


