from abc import abstractmethod

from contracts import ContractsMeta, contract

from blocks import SimpleBlackBox
from decent_logs import WithInternalLog
from streamels import BootSpec


__all__ = [
    'RepresentationNuisanceCausal',
]


class RepresentationNuisanceCausal(WithInternalLog):
    ''' 
        A Representation Nuisance that is a pair of dynamical systems:
        one before and one after the original system.
    '''
    
    __metaclass__ = ContractsMeta


    @abstractmethod
    def inverse(self):
        ''' 
            Returns the inverse representation nuisance,
            or raises NuisanceNotInvertible 
        '''
        raise NotImplementedError(type(self))

    @abstractmethod
    @contract(spec=BootSpec, returns=BootSpec)
    def transform_spec(self, spec):
        ''' 
        
        '''

    @contract(returns=SimpleBlackBox)
    def get_pre(self):
        return self.get_G()

    
    @contract(returns=SimpleBlackBox)
    def get_post(self):
        return self.get_H()
    
    # New clearer interface

    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_G(self):
        """ Returns G """ 
        
    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_H(self):
        """ 
            H and H* must take named streams "observations" and "commands".
            IF your H is simple you can use the following construction:
            
                H0 = ... # any SimpleBlackBox
                H0w = WrapTimedNamed(H0)
                H = Route([({'observations':'observations'}, H0w, {'observations':'observations'})])
        """
        
    
    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_G_conj(self):
        """ Returns G* """

    @abstractmethod
    @contract(returns=SimpleBlackBox)
    def get_H_conj(self):
        """ Returns H* """
