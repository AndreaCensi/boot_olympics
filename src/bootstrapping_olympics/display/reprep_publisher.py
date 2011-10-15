from ..interfaces import Publisher
from reprep import Report
from contextlib import contextmanager
from contracts import contract
from reprep import MIME_PYTHON

__all__ = ['ReprepPublisher']

class ReprepPublisher(Publisher):
   
    def __init__(self, rid=None, report=None):
        # TODO: clear up this interface
        if report is None:
            self.r = Report(rid)
        else:
            self.r = report
        self.f = self.r.figure()
    
    @contract(name='str', caption='None|str')
    def array(self, name, value, caption=None):
        self.f.data(name, value)

    @contract(name='str', value='array', filter='str', caption='None|str')
    def array_as_image(self, name, value, filter='posneg', filter_params={},
                       caption=None): #@ReservedAssignment
        if len(value.shape) == 3 and value.shape[2] == 3: # try image
            self.f.data_rgb(name, value)
        else:
            node = self.r.data(name, value, mime=MIME_PYTHON, caption=caption)
            m = node.display(filter, **filter_params)
            if caption is None:
                caption = name
            self.f.sub(m, caption=caption)
        
    @contract(name='str', value='str')
    def text(self, name, value):
        self.r.text(name, value)
        
    @contextmanager
    @contract(name='str', caption='None|str')
    def plot(self, name, caption=None, **args):
        with self.f.data_pylab(name, caption=caption, **args) as pylab:
            yield pylab
        
    def section(self, section_name):
        child = self.r.node(section_name)
        return ReprepPublisher(report=child)

