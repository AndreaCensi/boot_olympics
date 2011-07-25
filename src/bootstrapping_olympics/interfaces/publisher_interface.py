from contracts import contract
from abc import ABCMeta, abstractmethod

class Publisher:
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''

    __metaclass__ = ABCMeta
    

    @abstractmethod
    @contract(name='str', value='array')
    def array(self, name, value):
        ''' Publishes an array; the publisher decides how to visualize it. '''
    
    FILTER_POSNEG = 'posneg'
    FILTER_SCALE = 'scale'
    
    @abstractmethod
    @contract(name='str', value='array')
    def array_as_image(self, name, value, filter='posneg', filter_params={}):
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
    def plot(self, name, **args):
        ''' 
            Usage example: ::
        
                with publisher.plot('my plot') as pylab:
                    pylab.plot(Ey, label='E(y)')
                    pylab.plot(y_max, label='y_max')
                    pylab.plot(y_min, label='y_min')
                    pylab.legend()
        ''' 
    
