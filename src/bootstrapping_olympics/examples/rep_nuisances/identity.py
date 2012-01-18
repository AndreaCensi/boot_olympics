from ...interfaces import RepresentationNuisance

__all__ = ['Identity']


class Identity(RepresentationNuisance):

    ''' Encapsulates the idea of a representation nuisance,
        either on the observations or the commands. '''

    def inverse(self):
        return self

    def transform_spec(self, stream_spec):
        return stream_spec

    def transform_value(self, values):
        return values

    def __str__(self):
        return 'Identity'
