from abc import abstractmethod

from contracts import ContractsMeta, contract

from blocks import SimpleBlackBox
from decent_logs import WithInternalLog

from .boot_spec import BootSpec


__all__ = ['RepresentationNuisanceCausal']


class RepresentationNuisanceCausal(WithInternalLog):
    ''' 
        A Representation Nuisance that is a pair of dynamical systems:
        one before and one after the original system.
    '''
    
    __metaclass__ = ContractsMeta

    class NotInvertible(Exception):
        pass

    @abstractmethod
    def inverse(self):
        ''' 
            Returns the inverse representation nuisance,
            or raises NotInvertible 
        '''
        raise NotImplemented(type(self))

    @abstractmethod
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
