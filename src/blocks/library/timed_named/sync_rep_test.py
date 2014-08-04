from blocks.library import SyncRep
from blocks.unittests import BlocksTest


class SyncRepTest(BlocksTest):

    def sync_rep_test1(self):
        data = [
            (0.0, ('master', 'A')),
            (1.0, ('slave1', 'B')),
            (1.5, ('slave2', 'C')),
            (2.0, ('master', 'B')),
            (2.5, ('master', 'B')),
        ]
        expected = [
            (0.0, ('master', 'A')),
        ]
        bbox = SyncRep(master='master', slaves=['slave1', 'slave2'])
        self.check_bbox_results(bbox, data, expected)
        