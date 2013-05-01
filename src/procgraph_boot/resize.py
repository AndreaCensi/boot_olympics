from procgraph import simple_block
from procgraph_images.reshape_rect import reshape_rectangular
from procgraph_images.reshape_smart import reshape2d
from reprep.graphics import rgb_zoom
import numpy as np
from contracts import contract

@simple_block
def resize_if_necessary(rgb):
    """ 
    
        Resizes RGB images if they are too small; (for example,
        visualizing the commands would give you a 2x1 image.)
        
    """
    H, W, _ = rgb.shape
    
    M = max(H, W)
    
    min_M = 256
    min_factor = 2
    
    factor = int(np.ceil(min_M * 1.0 / M))
    
    factor = max(min_factor, factor)
    
    return rgb_zoom(rgb, factor)
    


@simple_block
@contract(max_ratio='float,>1')
def reshape_rectangular_soft(x0, max_ratio=1.5):
    """ 
        In contrast to reshape_rectangular, we do something
        sensible if the size is a prime number by calling reshape2d
        if the precise ratio cannot be achieved.
         
    """
    x = reshape_rectangular(x0)
    
    def get_ratio(shape):
        return np.max(shape) * 1.0 / np.min(shape)
     
    x_ratio = get_ratio(x.shape)
    if x_ratio > max_ratio:
        y = reshape2d(x, force=True)
    else:
        y = x
        
    # y_ratio = get_ratio(y.shape)
    # assert y_ratio <= max_ratio
    # print('x: %s ratio: %s y: %s ratio: %s' % (x.shape, x_ratio, y.shape, y_ratio))
    return y



