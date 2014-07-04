
from blocks.library import Collect

from .blocks_testing_utils import BlocksTest


class CollectTest(BlocksTest):

    def collect_test1(self):
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
        bbox = Collect()
        self.check_bbox_results(bbox, data, expected)
        

