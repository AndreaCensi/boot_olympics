from . import GenericLinear
from .. import contract, np

__all__ = ['GLNuisance']


class GLNuisance(GenericLinear):
    ''' A general linear transformation between R^n and R^n. '''

    @contract(A='array[NxN]|seq[N](seq[N])')
    def __init__(self, A):
        A = np.array(A)
        GenericLinear.__init__(self, A)
