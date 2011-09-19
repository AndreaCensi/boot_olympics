from ..interfaces import Publisher
from reprep import Report
from contextlib import contextmanager
from contracts import contract

__all__ = ['ReprepPublisher']

class ReprepPublisher(Publisher):
   
    def __init__(self, rid=None, report=None):
        # TODO: clear up this interface
        if report is None:
            self.r = Report(rid)
        else:
            self.r = report
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

    def array_as_image(self, name, value, filter='posneg', filter_params={}): #@ReservedAssignment
        f, name = self.get_sub(name)
        if len(value.shape) == 3 and value.shape[2] == 3: # try image
            f.data_rgb(name, value)
        else:
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
        
    def section(self, section_name):
        child = self.r.node(section_name)
        return ReprepPublisher(report=child)

