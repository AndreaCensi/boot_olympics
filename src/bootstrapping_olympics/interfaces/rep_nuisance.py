from abc import abstractmethod

__all__ = ['RepresentationNuisance']

# TODO: add "is_exact" to interface.

class RepresentationNuisance:
    
    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''
    
    @abstractmethod
    def inverse(self):
        ''' Returns the inverse representation nuisance. '''
        
    @abstractmethod
    def transform_spec(self, stream_spec):
        ''' Returns the changed StreamSpec, or throws
            UnsupportedSpec if the spec is not supported. '''
        
    @abstractmethod
    def transform_value(self, values):
        ''' Returns the transformed value. '''
        
