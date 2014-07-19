from blocks.interface import SimpleBlackBoxTN
from blocks.library.simple.instantaneous import Instantaneous
from blocks.library.timed.checks import check_timed_named


__all__ = ['IdentityTimedNamed']

    
class IdentityTimedNamed(Instantaneous, SimpleBlackBoxTN):

    def transform_value(self, value):
        check_timed_named(value)
        return value
        
    def __str__(self):
        return 'IdentityTimedNamed()'


