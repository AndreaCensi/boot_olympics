from ...interfaces import RepresentationNuisance

__all__ = ['Identity']


class Identity(RepresentationNuisance):
    ''' 
        The dummy identity nuisance. 
    
        There are some other checks useful for checking the protocl. 
    '''

    def __init__(self):
        self.transform_spec_called = False

    def transform_spec(self, stream_spec):
        self.transform_spec_called = True
        return stream_spec

    def inverse(self):
        if not self.transform_spec_called:
            msg = 'Calling inverse() before calling transform_spec().'
            raise ValueError(msg)
        return Identity()

    def transform_value(self, values):
        if not self.transform_spec_called:
            msg = 'Calling transform_value() before transform_spec().'
            raise ValueError(msg)
        return values.copy()

    def __str__(self):
        return 'Identity'
