from abc import abstractmethod
from contracts import (ContractsMeta, contract, describe_type, describe_value, 
    new_contract)
from contracts.utils import indent
from decent_logs import WithInternalLog
from streamels import StreamSpec
from streamels.base import check_valid_streamels
from streamels.exceptions import UnsupportedSpec
import traceback


__all__ = [
    'RepresentationNuisance',
    'NuisanceNotInvertible',
    'NuisanceNotLeftInvertible',
]

class NuisanceNotInvertible(ValueError):
    pass
class NuisanceNotLeftInvertible(ValueError):
    pass


class RepresentationNuisance(WithInternalLog):
    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''
    
    __metaclass__ = ContractsMeta

    @abstractmethod
    def inverse(self):
        ''' 
            Returns the inverse representation nuisance,
            or raises NuisanceNotInvertible.
            
            Note that this needs to be called after transform_spec().
            
            :raise NuisanceNotInvertible: The nuisance is not invertible.
        '''

    @abstractmethod
    def left_inverse(self):
        """ 
            Returns the left inverse representation nuisance
            or raises NuisanceNotInvertible.

            Note that this needs to be called after transform_spec().
            Defaults to calling left_inverse().
            
             :raise NuisanceNotInvertible: The nuisance is not left-invertible.
            
        """
        return self.inverse()

    def left_inverse_approx(self):
        """ 
            It should always return something -- cannot 
            raise NuisanceNotInvertible.
            
            The property is that the result will be the correct type
            but not necessarily a left inverse.
        """
        return self.left_inverse()

    @contract(stream_spec=StreamSpec, returns=StreamSpec)
    def transform_spec(self, stream_spec):
        ''' 
            Returns the changed StreamSpec, or throws
            UnsupportedSpec if the spec is not supported.
            
            You can either subclass this or :py:function:`transform_sensels`. 
        '''

        streamels = stream_spec.get_streamels()

        try:
            streamels2 = self.transform_streamels(streamels)
        except UnsupportedSpec:
            raise
        except BaseException as e:
            msg = 'Error while calling user-defined transform_streamels().'
            msg += '\n self: %s' % describe_value(self)
            msg += '\n of type %s' % describe_type(self)
            msg += '\n' + indent(traceback.format_exc(e), '| ')
            raise Exception(msg)
            

        try:
            check_valid_streamels(streamels2)
        except UnsupportedSpec:
            raise
        except ValueError as e:
            msg = 'User-defined transform_streamels() created invalid streamels.'
            msg += '\n self: %s' % describe_value(self)
            msg += '\n of type %s' % describe_type(self)
            msg += '\n' + indent(traceback.format_exc(e), '| ')
            raise Exception(msg)
            
        
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
