from bootstrapping_olympics.interfaces.publisher_interface import Publisher
from reprep import Report
from contextlib import contextmanager

class ReprepPublisher(Publisher):
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''
    
    def __init__(self, rid):
        self.r = Report(rid)
        self.f = self.r.figure(cols=3)
        
    def array(self, name, value):
        self.r.data(name, value)
        
    def array_as_image(self, name, value, filter='posneg', filter_params={}):
        self.r.data(name, value).display(filter, **filter_params)
        self.r.last().add_to(self.f)
        
    def text(self, name, value):
        self.r.text(name, value)
        
    @contextmanager
    def plot(self, name, **args):
        with self.r.data_pylab(name, **args) as pylab:
            yield pylab
        self.r.last().add_to(self.f)
        
