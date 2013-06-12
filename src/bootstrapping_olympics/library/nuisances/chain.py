from bootstrapping_olympics import RepresentationNuisance
from contracts import contract

__all__ = ['Chain']


class Chain(RepresentationNuisance):
    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''

    @contract(nuisances='list[>=2](RepresentationNuisance)')
    def __init__(self, nuisances):
        self.nuisances = nuisances
        
    def inverse(self):
        nuisances_inv = map(RepresentationNuisance.inverse, self.nuisances)
        return Chain(nuisances_inv[::-1])
                                                    
    def transform_spec(self, stream_spec):
        s2 = stream_spec
        for n in self.nuisances:
            s2 = n.transform_spec(s2)
        return s2
             
    def transform_value(self, values):
        v2 = values
        for n in self.nuisances:
            v2 = n.transform_value(v2)
        return v2

    @staticmethod
    @contract(specs='list(str|code_spec)')
    def instance_specs(specs):
        from bootstrapping_olympics import get_conftools_nuisances
        library = get_conftools_nuisances()
        inuis = lambda x: library.instance_smarter(x)[1] 
        nuisances = map(inuis, specs)
        if len(nuisances) == 1:
            return nuisances[0]
        else:
            return Chain(nuisances)
