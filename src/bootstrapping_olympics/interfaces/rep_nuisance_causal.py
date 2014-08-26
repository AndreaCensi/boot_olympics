from abc import abstractmethod
from blocks import SimpleBlackBoxT, SimpleBlackBoxTN
from blocks.library import InstantaneousTF, WrapTMfromT
from contracts import ContractsMeta, contract
from decent_logs import WithInternalLog
from streamels import BootSpec
from blocks.library.simple.identity import Identity


__all__ = [
    'RepresentationNuisanceCausal',
    'RepresentationNuisanceCausalSimple',
    'RepresentationNuisanceCausalSimpleInst',
    'RepresentationNuisanceCausalIdentity',
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
        try:
            g = self.get_g()
            G = InstantaneousTF(g)
            return G
        except:
            self.error('error in  %s:get_G()' % type(self).__name__)
            raise
        
        
    
    @contract(returns=SimpleBlackBoxT)
    def get_G_conj(self):
        return InstantaneousTF(self.get_g_conj())
    
    


class RepresentationNuisanceCausalIdentity(RepresentationNuisanceCausalSimpleInst):
    """ Further special case where G and H are implemented by 
        some instantaneous functions get_h and get_g. """
        
    def get_g(self):
        return lambda x: x

    def get_g_conj(self):
        return lambda x: x

    def get_h(self):
        return lambda x: x
        
    def get_h_conj(self):
        return lambda x: x

    def inverse(self):
        return RepresentationNuisanceCausalIdentity()
    
    def transform_spec(self, spec):
        return spec
    
    