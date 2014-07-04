from unittest.case import TestCase

from contracts import contract

from blocks import SimpleBlackBox
from blocks import series
from blocks.library import FromData
from blocks.pumps import source_read_all_block


__all__ = ['BlocksTest']


class BlocksTest(TestCase):

    @contract(bbox=SimpleBlackBox, data=list, expected=list)
    def check_bbox_results(self, bbox, data, expected):
        s = series(FromData(data), bbox)
        s.reset()
        res = source_read_all_block(s)
        self.assert_same_seq(res, expected)

    def assert_same_seq(self, a, b):
        try:
            self.assertEqual(a, b)
        except:
            print('result:')
            self.display_signal(a)
            print('expected:')
            self.display_signal(b)
            raise
    
    def display_signal(self, x):
        for (t, (name, obs)) in x:
            print('%10s %10s %s' % (t,name, obs))
        
