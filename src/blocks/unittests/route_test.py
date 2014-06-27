from unittest.case import TestCase

from blocks.composition import series
from blocks.library.collect import Collect
from blocks.library.from_data import FromData
from blocks.pumps import source_read_all_block
from blocks.library.delay import Delay
from blocks.library.identity import Identity
from blocks.library.route import Route


class RouteTest(TestCase):

    def from_data_test1(self):
        data = [
            (0.0, ('a', 'A0')),
            (1.0, ('a', 'A1')),
            (1.0, ('b', 'B1')),
            (2.0, ('b', 'B2')),
            (3.0, ('a', 'A3')),
            (3.0, ('b', 'B3')),
        ]
        d = 0.1

        expected = [
            (0.0, ('c', 'A0')),
            (0.0 + d, ('a', 'A0')),
            (1.0, ('c', 'A1')),
            (1.0, ('d', 'B1')),
            (1.0 + d, ('a', 'A1')),
            (2.0, ('d', 'B2')),
            (3.0, ('c', 'A3')),
            (3.0, ('d', 'B3')),
            (3.0 + d, ('a', 'A3')),
        ]

        routing = [
           # a is delayed
           ({'a':'a'}, Delay(d), {'a':'a'}),
           # a becomes 'c' at input,
           # b becomes 'd' at output
           ({'a':'c', 'b':'b'}, Identity(), {'c':'c', 'b':'d'}),
        ]

        r = Route(routing)
        s = series(FromData(data), r)
        res = source_read_all_block(s)
        self.assertEqual(res, expected)


