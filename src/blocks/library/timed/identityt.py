from blocks.library.simple.instantaneous import Instantaneous
from blocks.library.timed.checks import check_timed
from blocks.interface import SimpleBlackBoxT


__all__ = ['IdentityTimed']

    
class IdentityTimed(Instantaneous, SimpleBlackBoxT):

    def transform_value(self, value):
        check_timed(value, self)
        return value
        
    def __repr__(self):
        return 'IdentityTimed()'


