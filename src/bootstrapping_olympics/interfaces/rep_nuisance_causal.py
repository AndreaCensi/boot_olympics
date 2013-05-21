from contracts.metaclass import ContractsMeta
from bootstrapping_olympics.interfaces.boot_spec import BootSpec
from contracts import contract
from blocks import SimpleBlackBox
from abc import abstractmethod
    
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
