from . import contract
from ..interfaces import Publisher
from contextlib import contextmanager
from reprep import MIME_PYTHON, Report
 
__all__ = ['ReprepPublisher']

class ReprepPublisher(Publisher):
   
    def __init__(self, rid=None, report=None, cols=None):
        # TODO: clear up this interface
        if report is None:
            self.r = Report(rid)
        else:
            self.r = report
            
        self.cols = cols
        self._f = None
    
    def fig(self):
        ''' Returns reference to current RepRep figure. '''
        if self._f is None:
            self._f = self.r.figure(cols=self.cols)
        return self._f
    
    @contract(name='str', caption='None|str')
    def array(self, name, value, caption=None): # XXX to change
        self.fig().data(name, value)

    @contract(name='str', value='array', filter='str', caption='None|str')
    def array_as_image(self, name, value, filter='posneg', filter_params={}, #@ReservedAssignment
                       caption=None): #@ReservedAssignment
        if len(value.shape) == 3 and value.shape[2] == 3: # try image XXX check uint8
            # zoom images smaller than 50
#            if value.shape[0] < 50:
#                value = zoom(value, 10)
            self.fig().data_rgb(name, value)
        else:
            node = self.r.data(name, value, mime=MIME_PYTHON, caption=caption)
            m = node.display(filter, **filter_params)
            if caption is None:
                caption = name
            self.fig().sub(m, caption=caption)
        
    @contract(name='str', value='str')
    def text(self, name, value):
        self.r.text(name, value)
        
    @contextmanager
    @contract(name='str', caption='None|str')
    def plot(self, name, caption=None, **args):
        with self.fig().data_pylab(name, caption=caption, **args) as pylab:
            yield pylab
        
    def section(self, section_name, cols=None):
        child = self.r.node(section_name)
        return ReprepPublisher(report=child, cols=cols)

