from bootstrapping_olympics.configuration.master import get_boot_config
from bootstrapping_olympics.interfaces import StreamSpec
from bootstrapping_olympics.interfaces.agent import UnsupportedSpec
from bootstrapping_olympics.interfaces.streamels import make_streamels_float
from procgraph import Block
import warnings


class ApplyNuisance(Block):
    """ Applies a nuisance to the signal. """ 
    Block.alias('apply_nuisance')
    Block.config('nuisance', 'ID of nuisance') 

    Block.input('y0')
    
    Block.output('y1')
    
    def init(self):
        id_nuisance = self.config.nuisance
        boot_config = get_boot_config()
        self.nuisance = boot_config.nuisances.instance(id_nuisance) 
        self.y0_spec = None
        
        self.failed = False
        
    def update(self):
        if self.failed:
            return
        
        warnings.warn('here we are only guessing a spec; make it more formal')
        y0 = self.input.y0
        if self.y0_spec is None:
            # FIXME
            lower = 0
            upper = 1
            shape = y0.shape
            streamels = make_streamels_float(shape, lower, upper)
            self.y0_spec = StreamSpec(id_stream='tmp', streamels=streamels, extra={})
            try:
                self.y1_spec = self.nuisance.transform_spec(self.y0_spec)
            except UnsupportedSpec as e:
                self.info('Not writing anything because failed: %s' % e)
                self.failed = True
                return
            
        y1 = self.nuisance.transform_value(y0)
        
        self.output.y1 = y1
        
