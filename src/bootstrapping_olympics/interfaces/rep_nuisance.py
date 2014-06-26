from abc import abstractmethod

from contracts import contract, ContractsMeta, new_contract

from decent_logs import WithInternalLog

from .streamels import StreamSpec


__all__ = ['RepresentationNuisance', 'NuisanceNotInvertible']

class NuisanceNotInvertible(ValueError):
    pass


class RepresentationNuisance(WithInternalLog):
    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''
    
    __metaclass__ = ContractsMeta


    @abstractmethod
    def inverse(self):
        ''' 
            Returns the inverse representation nuisance,
            or raises NuisanceNotInvertible 
        '''

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

    @contract(streamels='streamel_array', returns='streamel_array')
    def transform_streamels(self, streamels):
        raise NotImplementedError()

    @abstractmethod
    def transform_value(self, values):
        ''' Returns the transformed value. '''

new_contract('RepresentationNuisance', RepresentationNuisance)
