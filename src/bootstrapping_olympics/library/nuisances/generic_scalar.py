from abc import abstractmethod

from bootstrapping_olympics import (RepresentationNuisance,
    check_streamels_continuous)


__all__ = ['GenericScalar']


class GenericScalar(RepresentationNuisance):
    ''' 
        This represents a generic nonlinear scalar transformation
        that acts on each sensel individually. 
    '''

    @abstractmethod
    def transform01(self, x):
        """ 
            Must be implemented by subclasses. 
            x can be an array of arbitrary shape with values in [0,1] 
        """
        
    def transform_value(self, x):
        L = self.streamels['lower']
        U = self.streamels['upper']
        W = U - L
        y = (x - L) / W
        z = self.transform01(y)
        return L + z * W 

    # FIXME: must remap to [0,1]
    
    def transform_streamels(self, streamels):
        check_streamels_continuous(streamels)
        self.streamels = streamels.copy()
        return self.streamels

