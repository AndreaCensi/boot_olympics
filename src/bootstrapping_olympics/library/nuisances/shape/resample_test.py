import numpy as np
import unittest
from bootstrapping_olympics.library.nuisances.shape.resample import scipy_image_resample
from bootstrapping_olympics.utils import assert_allclose

H1, W1 = (10, 20)
H2, W2 = (10, 10)
        
        
class TestResample(unittest.TestCase):
        
    def scipy_image_resample_test1(self):        
        # uint8, RGB
        a = np.zeros((H1, W1, 3), 'uint8')
        b = scipy_image_resample(a, (H2, W2))
        print b.shape, b.dtype
        self.assertEqual(b.shape, (H2, W2, 3))
        self.assertEqual(b.dtype, 'uint8')
        
    def scipy_image_resample_test2(self):
        # float32, 3 channel
        a = np.zeros((H1, W1, 3), 'float32') 
        b = scipy_image_resample(a, (H2, W2))
        print b.shape, b.dtype
        self.assertEqual(b.shape, (H2, W2, 3))
        self.assertEqual(b.dtype, 'float32')
        
    def scipy_image_resample_test3(self):
        # float32, 1 channel
        a = np.zeros((H1, W1), 'float32') 
        b = scipy_image_resample(a, (H2, W2))
        print b.shape, b.dtype
        self.assertEqual(b.shape, (H2, W2))
        self.assertEqual(b.dtype, 'float32')

    def scipy_image_resample_test4(self):
        """ Resampling should be invertible if using order = 0 """
        a = np.random.rand(10, 10).astype('float32')
        b = scipy_image_resample(a, (20, 20), order=0)
        a1 = scipy_image_resample(b, (10, 10), order=0)
        assert_allclose(a1, a)
        
