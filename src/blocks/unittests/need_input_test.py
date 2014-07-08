
from blocks import NeedInput, Finished
from blocks.composition import series
from blocks.library import Identity

from .blocks_testing_utils import BlocksTest


class NeedInputTest(BlocksTest):

    def need_input_test1(self):

        bb = Identity()
        bb.reset()

        self.assertRaises(NeedInput, bb.get, block=True)
        bb.put(42)
        self.assertEqual(bb.get(block=True), 42)

        bb.end_input()
        self.assertRaises(Finished, bb.get, block=True)

    def need_input_test2(self):

        bb = series(Identity(), Identity())
        bb.reset()

        self.assertRaises(NeedInput, bb.get, block=True)
        bb.put(42)
        self.assertEqual(bb.get(block=True), 42)

        bb.end_input()
        self.assertRaises(Finished, bb.get, block=True)

    def need_input_test3(self):

        bb = series(Identity(), Identity(), Identity())
        bb.reset()

        self.assertRaises(NeedInput, bb.get, block=True)
        bb.put(42)
        self.assertEqual(bb.get(block=True), 42)

        bb.end_input()
        self.assertRaises(Finished, bb.get, block=True)
