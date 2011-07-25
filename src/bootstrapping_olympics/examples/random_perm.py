from bootstrapping_olympics.interfaces.observations_filter_interface import  FilterInterface
import numpy as np

class RandomPermutation(FilterInterface):

    def __init__(self, seed):
        self.seed = seed
        self.permutation = None
             
    def process(self, sensel_values):
        if self.permutation is None:
            n = len(sensel_values)
            np.random.seed(self.seed)
            self.permutation = random_permutation(n)
        
        return sensel_values[self.permutation]
            

def random_permutation(n):
    ''' returns a random permutation of n elements '''
    return np.argsort(np.random.rand(n))
