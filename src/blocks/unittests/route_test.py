from unittest.case import TestCase

from blocks import series
from blocks.library import Delay
from blocks.library import FromData
from blocks.library import Identity
from blocks.library import Route
from blocks.pumps import source_read_all_block


class RouteTest(TestCase):

    def route_test1(self):
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
            # Note: this Route cannot compensate delays
            (1.0 + d, ('a', 'A1')),
            (1.0, ('d', 'B1')),
            (2.0, ('d', 'B2')),
            (3.0, ('c', 'A3')),
            # Note: this Route cannot compensate delays
            (3.0 + d, ('a', 'A3')),
            (3.0, ('d', 'B3')),
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
        
