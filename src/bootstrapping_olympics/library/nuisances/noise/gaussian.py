from .. import contract, np
from ..generic_scalar import GenericScalar
from bootstrapping_olympics import NuisanceNotInvertible


__all__ = ['Gaussian']


class Gaussian(GenericScalar):

    """ Additive Gaussian noise. """

    @contract(sigma='>0')
    def __init__(self, sigma):
        self.sigma = sigma
        
    def inverse(self):
        msg = "Gaussian noise is not invertible"
        raise NuisanceNotInvertible(msg)

    def transform01(self, values01):
        noise = np.random.randn(*values01.shape) * self.sigma
        y = values01 + noise
        return np.clip(y, 0.0, 1.0)

