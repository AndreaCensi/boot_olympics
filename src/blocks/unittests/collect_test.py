from unittest.case import TestCase

from blocks.composition import series
from blocks.library.collect import Collect
from blocks.library.from_data import FromData
from blocks.pumps import source_read_all_block


class CollectTest(TestCase):

    def from_data_test1(self):
        data = [
            (0.0, ('a', 'A')),
            (1.0, ('b', 'B')),
            (1.0, ('c', 'C2')),
            (2.0, ('b', 'B2')),
            (3.0, ('a', 'A3')),
            (3.0, ('b', 'B3')),
            (3.0, ('c', 'C3')),
        ]
        expected = [
            (0.0, dict(a='A')),
            (1.0, dict(b='B', c='C2')),
            (2.0, dict(b='B2')),
            (3.0, dict(a='A3', b='B3', c='C3')),
        ]
        s = series(FromData(data), Collect())
        res = source_read_all_block(s)
        self.assertEqual(res, expected)


