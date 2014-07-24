from contracts  import contract
import numpy as np 
from bootstrapping_olympics import NuisanceNotInvertible
from bootstrapping_olympics.library.nuisances import GenericScalar
from bootstrapping_olympics.interfaces.rep_nuisance import NuisanceNotLeftInvertible


__all__ = ['Gaussian']


class Gaussian(GenericScalar):

    """ Additive Gaussian noise. """

    @contract(sigma='>0')
    def __init__(self, sigma):
        self.sigma = sigma
        
    def inverse(self):
        msg = "Gaussian noise is not invertible"
        raise NuisanceNotInvertible(msg)

    def left_inverse(self):
        raise NuisanceNotLeftInvertible() 

    def transform01(self, values01):
        noise = np.random.randn(*values01.shape) * self.sigma
        y = values01 + noise
        return np.clip(y, 0.0, 1.0)

