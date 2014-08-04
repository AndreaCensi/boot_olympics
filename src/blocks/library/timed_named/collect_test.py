
from blocks import NeedInput
from blocks.library import Collect, CollectSignals
from blocks.unittests import BlocksTest

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
        


    def collect_test2(self):
        collect = Collect()
        collect.reset()
        collect.put((0, ('a', 'A0')))
        collect.put((0, ('b', 'B0')))
        self.assertRaises(NeedInput, collect.get)
        collect.put((1, ('a', 'Aa')))
        self.assertEqual(collect.get(), (0, dict(a='A0', b='B0')))
        self.assertRaises(NeedInput, collect.get)

    def collect_test3(self):
        collect = CollectSignals(signals=set(['a', 'b']))
        collect.reset()
        collect.put((0, ('a', 'A0')))
        collect.put((0, ('b', 'B0')))
        self.assertEqual(collect.get(), (0, dict(a='A0', b='B0')))
