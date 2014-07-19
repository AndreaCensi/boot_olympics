from abc import abstractmethod
from blocks import SimpleBlackBox
from blocks.library import InstantaneousF, WrapT
from contracts import ContractsMeta, contract
from decent_logs import WithInternalLog
from streamels import BootSpec
from blocks.library.timed_named.wrappers import WrapTimedNamed, WrapTMfromT
from blocks.interface import SimpleBlackBoxT, SimpleBlackBoxTN
from blocks.library.simple.instantaneous import InstantaneousTF




__all__ = [
    'RepresentationNuisanceCausal',
    'RepresentationNuisanceCausalSimple',
    'RepresentationNuisanceCausalSimpleInst',
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

    @abstractmethod
    @contract(returns=SimpleBlackBoxT)
    def get_G(self):
        """ Returns G -- must take inputs of the kind (time, value) """ 
        
    @abstractmethod
    @contract(returns=SimpleBlackBoxTN)
    def get_L(self):
        """ 
            L and L* must take named streams "observations" and "commands".
            IF your H is simple you can use the following construction:
            
                H0 = ... # any SimpleBlackBox
                H0w = WrapTimedNamed(H0)
                H = Route([({'observations':'observations'}, H0w, {'observations':'observations'})])
        """
    
    @abstractmethod
    @contract(returns=SimpleBlackBoxT)
    def get_G_conj(self):
        """ Returns G* """

    @abstractmethod
    @contract(returns=SimpleBlackBoxTN)
    def get_L_conj(self):
        """ Returns H* """
            

class RepresentationNuisanceCausalSimple(RepresentationNuisanceCausal):
    
    """ In this case, the L box does not depend on the commands. """
     
    @abstractmethod
    @contract(returns=SimpleBlackBoxT)
    def get_H(self):
        """ A SimpleBlackBox that takes (time, value) as input. """
    
    @abstractmethod
    @contract(returns=SimpleBlackBoxT)
    def get_H_conj(self):
        """ A SimpleBlackBox that takes (time, value) as input. """
    
    @contract(returns=SimpleBlackBoxTN)
    def get_L(self):
        from blocks.library import Route
        H0 = self.get_H()
        H = WrapTMfromT(H0)
        r = Route([({'observations':'observations'},
                    H,
                    {'observations':'observations'})],
                  suppress=['commands'])
        return r
    
    @contract(returns=SimpleBlackBoxTN)
    def get_L_conj(self):
        from blocks.library import Route
        
        H0_conj = self.get_H_conj()
        H_conj = WrapTMfromT(H0_conj)
        r = Route([({'observations': 'observations'},
                    H_conj, 
                    {'observations': 'observations'})],
                  suppress=['commands'])
        return r
    
class RepresentationNuisanceCausalSimpleInst(RepresentationNuisanceCausalSimple):
    """ Further special case where G and H are implemented by 
        some instantaneous functions get_h and get_g. """
        
    @abstractmethod
    def get_g(self):
        """ returns an instantaneous function """

    @abstractmethod
    def get_g_conj(self):
        """ returns an instantaneous function """

    @abstractmethod
    def get_h(self):
        """ returns an instantaneous function """
        
    @abstractmethod
    def get_h_conj(self):
        """ returns an instantaneous function """
        
    @contract(returns=SimpleBlackBoxT)
    def get_H(self):
        return InstantaneousTF(self.get_h())
    
    @contract(returns=SimpleBlackBoxT)
    def get_H_conj(self):
        return InstantaneousTF(self.get_h_conj())
    
    @contract(returns=SimpleBlackBoxT)
    def get_G(self):
        return InstantaneousTF(self.get_g())
    
    @contract(returns=SimpleBlackBoxT)
    def get_G_conj(self):
        return InstantaneousTF(self.get_g_conj())
    
    
