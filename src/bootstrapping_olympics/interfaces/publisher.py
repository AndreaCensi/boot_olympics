from . import contract
from abc import ABCMeta, abstractmethod
from contextlib import contextmanager


__all__ = ['Publisher'] 

class Publisher:
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''

    __metaclass__ = ABCMeta
    

    @abstractmethod
    @contract(name='str', value='array')
    def array(self, name, value, caption=None):
        ''' Publishes an array; the publisher decides how to visualize it. '''
    
    FILTER_POSNEG = 'posneg'
    FILTER_SCALE = 'scale'
    
    @abstractmethod
    @contract(name='str', value='array')
    def array_as_image(self, name, value,
                       filter='posneg', filter_params={}, #@ReservedAssignment
                       caption=None): 
        ''' 
            Publishes an array as a false-color image. 
            ``filter`` is the name of a filter.
            ``filter_params`` are parameters to pass to the filter.
            
            Usage example: ::
            
                publisher.array_as_image('covariance', P, 'posneg')
        '''
    
    @abstractmethod
    @contract(name='str', text='str')
    def text(self, name, text):
        ''' 
            Publishes a text object.

            Usage example: ::
            
                publisher.text('status', 'I am ok')
        '''
    
    @abstractmethod
    def plot(self, name, caption=None, **args):
        ''' 
            Usage example: ::
        
                with publisher.plot('my plot') as pylab:
                    pylab.plot(Ey, label='E(y)')
                    pylab.plot(y_max, label='y_max')
                    pylab.plot(y_min, label='y_min')
                    pylab.legend()
        ''' 
    # TODO: make this abstract   
    def section(self, section_name):
        return Section(self, section_name)


class Section():
    def __init__(self, other, prefix):
        self.other = other
        self.prefix = prefix
    def concat(self, name):
        base = [self.prefix]
        if isinstance(name, str):
            base.append(name)
        else:
            base.extend(name)
        return "-".join(base)
    def array(self, name, *args):
        return self.other.array(self.concat(name), *args)
    def array_as_image(self, name, *args, **kwargs):
        return self.other.array_as_image(self.concat(name), *args, **kwargs)
    def text(self, name, text):
        return self.other.text(self.concat(name), text)
    @contextmanager
    def plot(self, name, **args):
        with self.other.plot(self.concat(name), **args) as pylab:
            yield pylab
    def section(self, section_name):
        return Section(self, section_name)

