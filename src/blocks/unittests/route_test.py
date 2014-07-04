from blocks.library import Delay, Identity, Route

from .blocks_testing_utils import BlocksTest


class RouteTest(BlocksTest):

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

        bbox = Route(routing)

        self.check_bbox_results(bbox, data, expected)
