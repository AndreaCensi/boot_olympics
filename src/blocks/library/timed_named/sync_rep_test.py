from blocks.library import SyncRep
from blocks.unittests import BlocksTest


class SyncRepTest(BlocksTest):

    def sync_rep_test1(self):
        data = [
            (0.0, ('master', 'A')),
            (1.0, ('slave1', 'B')),
            (1.5, ('slave2', 'C')),
            (2.0, ('master', 'B')),
            (2.1, ('slave2', 'D')),
            (2.5, ('master', 'B')),
        ]
        expected = [
            (0.0, ('master', 'A')),
            (2.0, ('master', 'B')),
            (2.0, ('slave1', 'B')),
            (2.0, ('slave2', 'C')),
            (2.5, ('master', 'B')),
            (2.5, ('slave1', 'B')),
            (2.5, ('slave2', 'D')),
        ]
        bbox = SyncRep(master='master')
        self.check_bbox_results(bbox, data, expected)
        