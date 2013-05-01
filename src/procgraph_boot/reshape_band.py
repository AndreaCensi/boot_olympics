from procgraph import Block
import numpy as np


class ReshapeBand(Block):
    """ Creates a band-version for 1D signals. No-op for 2D signals. """
     
    Block.alias('reshape_band')
    Block.config('height', default=64) 

    Block.input('y0')
    Block.output('y1')
    
    def init(self):
        pass
        
    def update(self):
        y0 = self.input.y0
        if y0.ndim != 1:
            return
        
        height = self.config.height
        
        y1 = np.tile(y0, (height, 1))
        assert y1.shape == (height, y0.size)
                        
        self.output.y1 = y1
        
