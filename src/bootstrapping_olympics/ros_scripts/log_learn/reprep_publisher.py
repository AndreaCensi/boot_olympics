from bootstrapping_olympics.interfaces.publisher_interface import Publisher
from reprep import Report
from contextlib import contextmanager
from contracts import contract

class ReprepPublisher(Publisher):
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''
    
    def __init__(self, rid):
        self.r = Report(rid)
        self.subs = {}
    
    @contract(name='str|tuple[=2]')
    def get_sub(self, name):
        if isinstance(name, str):
            subname = 'default'
            resname = name
        elif isinstance(name, tuple):
            subname = name[0]
            resname = name[1]
        if not subname in self.subs:
            self.subs[subname] = self.r.figure(subname, cols=4)
        return self.subs[subname], resname
    
    def array(self, name, value):
        f, name = self.get_sub(name)
        f.data(name, value)
        
    def array_as_image(self, name, value, filter='posneg', filter_params={}):
        f, name = self.get_sub(name)
        f.data(name, value).display(filter, **filter_params)
        f.last().add_to(f) # XXX
        
    def text(self, name, value):
        f, name = self.get_sub(name)
        f.text(name, value)
        
    @contextmanager
    def plot(self, name, **args):
        f, name = self.get_sub(name)
        with f.data_pylab(name, **args) as pylab:
            yield pylab
        f.last().add_to(f)
        
