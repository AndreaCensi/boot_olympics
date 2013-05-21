from .boot_spec import BootSpec
from abc import abstractmethod
from blocks import SimpleBlackBox
from contracts import ContractsMeta, contract
    
__all__ = ['RepresentationNuisanceCausal']


class RepresentationNuisanceCausal(object):
    ''' 

    '''
    
    __metaclass__ = ContractsMeta

    class NotInvertible(Exception):
        pass

    def inverse(self):
        ''' 
            Returns the inverse representation nuisance,
            or raises NotInvertible 
        '''

    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        ''' 
        
        '''

    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_pre(self):
        pass
    
    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_post(self):
        pass
