from blocks import SimpleBlackBox, series
from blocks.library import FromData
from blocks.pumps import source_read_all_block
from contracts import contract
from contracts.utils import indent
from unittest.case import TestCase


__all__ = [
    'BlocksTest',
]


class BlocksTest(TestCase):

    @contract(bbox=SimpleBlackBox, data=list, expected=list)
    def check_bbox_results(self, bbox, data, expected):
        res = self.get_bbox_results(bbox, data)
        self.assert_same_seq(res, expected)

    def get_bbox_results(self, bbox, data):
        s = series(FromData(data), bbox)
        s.reset()
        res = source_read_all_block(s)
        return res

    def assert_same_seq(self, a, b):

        def dl(a):
            return '\n'.join([str(x) for x in a])

        if not (a == b):
            print('result:')
            self.display_signal(a)
            print('expected:')
            self.display_signal(b)
#             self.assertEqual(a, b)
            msg = 'sequences dont match'
            msg += '\n obtained:\n' + indent(dl(a), '| ')
            msg += '\n expected:\n' + indent(dl(b), '| ')
            raise ValueError(msg)



#         try:
#             self.assertEqual(a, b)
#         except:
#             print('result:')
#             self.display_signal(a)
#             print('expected:')
#             self.display_signal(b)
#             raise
    
    def display_signal(self, x):
        if len(x) == 0:
            print(' (empty sequence) ')
        for v in x:
            print(str(v))
#
#         for (t, (name, obs)) in x:
#             print('%10s %10s %s' % (t,name, obs))
#
