from blocks.unittests import BlocksTest
from .sampled_deriv import SampledDerivInst, SampledDeriv


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
        

    def sampled_deriv_inst_test1(self):
        # The Inst variant needs to return instantaneously 
        bbox = SampledDerivInst()
        bbox.reset()
        bbox.put((0.0, 10.0))
        x = bbox.get(block=True)
        self.assertEqual(x, (0.0, (10.0, 0.0)))
        
        
    def sampled_deriv_inst_test2(self):
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
            (0.0, (10.0, +0.0)),
            (1.0, (11.0, +1.0)),
            (2.0, (12.0, +1.0)),
            (3.0, (13.0, +1.0)),
            #(4.0, (14.0, +0.5)), # other behavior
            (4.0, (14.0, +1.0)),
            (5.0, (14.0, +0.0)),
            (6.0, (14.0, +0.0)),
        ]
        bbox = SampledDerivInst()
        self.check_bbox_results(bbox, data, expected)

