from . import StreamSpec, contract
from abc import abstractmethod, ABCMeta

__all__ = ['RepresentationNuisance', 'NuisanceNotInvertible']


class RepresentationNuisance():
    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''
    # TODO: add "is_exact" to interface.
    __metaclass__ = ABCMeta


    @abstractmethod
    def inverse(self):
        ''' Returns the inverse representation nuisance, or 
            NuisanceNotInvertible '''

    @contract(streamels='streamel_array', returns='streamel_array')
    def transform_streamels(self, streamels):
        raise NotImplementedError()

    @abstractmethod
    def transform_value(self, values):
        ''' Returns the transformed value. '''

    @contract(stream_spec=StreamSpec, returns=StreamSpec)
    def transform_spec(self, stream_spec):
        ''' 
            Returns the changed StreamSpec, or throws
            UnsupportedSpec if the spec is not supported.
            
            You can either subclass this or :py:function:`transform_sensels`. 
        '''

        streamels = stream_spec.get_streamels()

        streamels2 = self.transform_streamels(streamels)

        stream_spec2 = StreamSpec(id_stream=stream_spec.id_stream,
                                  streamels=streamels2,
                                  extra={},  # TODO: annotate
                                  filtered={},
                                  desc=stream_spec.desc)
        return stream_spec2


class NuisanceNotInvertible(ValueError):
    pass

