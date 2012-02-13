from abc import abstractmethod, ABCMeta

__all__ = ['RepresentationNuisance', 'NuisanceNotInvertible']

# TODO: add "is_exact" to interface.


class NuisanceNotInvertible(ValueError):
    pass


class RepresentationNuisance:
    __metaclass__ = ABCMeta

    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''

    @abstractmethod
    def inverse(self):
        ''' Returns the inverse representation nuisance, or 
            NuisanceNotInvertible '''

    @abstractmethod
    def transform_spec(self, stream_spec):
        ''' Returns the changed StreamSpec, or throws
            UnsupportedSpec if the spec is not supported. '''

    @abstractmethod
    def transform_value(self, values):
        ''' Returns the transformed value. '''

