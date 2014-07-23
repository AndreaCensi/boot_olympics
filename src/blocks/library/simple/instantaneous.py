from .with_queue import WithQueue
from abc import abstractmethod
from contracts import describe_value, describe_type
from blocks import SimpleBlackBoxT


__all__ = [
    'Instantaneous',
    'InstantaneousF',
    'InstantaneousTF',  
]

class Instantaneous(WithQueue):

    def __init__(self):
        WithQueue.__init__(self)

    @abstractmethod
    def transform_value(self, value):
        pass

    def put_noblock(self, value):
        try:
            value2 = self.transform_value(value)
        except BaseException as e:
#             raise_wrapped(type(e), e, 
#
            msg =  'Error while calling user method transform_value().' 
            msg += '\n   this block: %s' %  describe_value(self)
            msg += '\n  of type %s' %  describe_type(self)
            msg += '\n  value: %s' %  describe_value(value)
            msg += '\n  of type %s' %  describe_type(value)

            self.error(msg)
            raise
        self.append(value2)

class InstantaneousF(Instantaneous):
    """ Instantaneous transform with given function. """

    def __init__(self, func):
        WithQueue.__init__(self)
        self.func = func

    def transform_value(self, value):
        return self.func(value)



class InstantaneousTF(Instantaneous, SimpleBlackBoxT):
    """ Instantaneous transform with given function. """

    def __init__(self, func):
        WithQueue.__init__(self)
        self.func = func

    def transform_value(self, value):
        from blocks.library.timed.checks import check_timed
        from blocks.library.timed.checks import check_timed_named

        try:
            check_timed_named(value)
        except:
            pass
        else:
            msg = 'I did not expect this to be also named...'
            msg += '\nvalue: %s' % describe_value(value)
            self.error(msg)
            raise ValueError(msg)

        check_timed(value)
        t, ob = value
        
        try:
            ob2 = self.func(ob)
        except BaseException:
            msg = 'Error while calling user function self.func in transform_value().'
            msg += '\n  passed: %s' %  describe_value(ob)
            msg += '\n  of type %s' %  describe_type(ob)
            msg += '\n  user func: %s' %  self.func
            self.error(msg)
            raise
      
            raise
        value2 = t, ob2
        return value2

