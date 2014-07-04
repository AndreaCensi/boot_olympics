from blocks.library import SampledDeriv

from .blocks_testing_utils import BlocksTest


class SampledDerivTest(BlocksTest):

    def sampled_deriv_test1(self):
        data = [
            (0.0, 10.0),
            (1.0, 11.0),
            (2.0, 12.0),
            (3.0, 13.0),
            (4.0, 14.0),
            (5.0, 14.0),
            (6.0, 14.0),
        ]
        expected = [
            (1.0, (11.0, +1)),
            (2.0, (12.0, +1)),
            (3.0, (13.0, +1)),
            (4.0, (14.0, +0.5)),
            (5.0, (14.0, +0.0)),
        ]
        
        bbox = SampledDeriv()
        
        self.check_bbox_results(bbox, data, expected)
        
