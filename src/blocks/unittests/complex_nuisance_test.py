from blocks.composition import series
from blocks.library import FromData, Identity, WithQueue, ToData
from blocks.pumps import source_read_all_block
import numpy as np

from .blocks_testing_utils import BlocksTest
from blocks.utils import check_reset


class SuperSample(WithQueue):

    def __init__(self, min_interval):
        """ This guarantees that the output
            is at most min_interval. """
        WithQueue.__init__(self)
        self.min_interval = min_interval

    def reset(self):
        self.last = None

    def put_noblock(self, value):
        check_reset(self, 'last')

        if self.last is not None:
            t1, v1 = self.last
            t2, _ = value
            if t2 - t1 > self.min_interval:
                n = np.ceil((t2 - t1) / self.min_interval)
                n = int(n)
                for i in range(n):
                    t = t1 + (i) * self.min_interval
                    v = v1
                    self.append((t, v))
        self.last = value

    def end_input(self):
        check_reset(self, 'last')

        if self.last is not None:
            self.append(self.last)
        WithQueue.end_input(self)




class ConnectionTests(BlocksTest):

    def from_data_test1(self):
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        s = FromData(data)
        s.reset()
        res = source_read_all_block(s)
        self.assertEqual(data, res)

    def series_test_1(self):
        """  Series(FromDatA, Identity) """
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        bbox = Identity()
        expected = data
        self.check_bbox_results(bbox, data, expected)


    def series_test_2(self):
        """  Series(FromDatA, SuperSample) """
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        expected = [(0.0, 'A'), (0.25, 'A'), (0.5, 'A'), (0.75, 'A'),
               (1.0, 'B'), (1.25, 'B'), (1.5, 'B'), (1.75, 'B'),
               (2.0, 'C')]
        bbox = SuperSample(0.25)
        self.check_bbox_results(bbox, data, expected)

    def series_test_3(self):
        """  Series(SuperSample, ToData) """
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        out = [(0.0, 'A'), (0.25, 'A'), (0.5, 'A'), (0.75, 'A'),
               (1.0, 'B'), (1.25, 'B'), (1.5, 'B'), (1.75, 'B'),
               (2.0, 'C')]
        f = SuperSample(0.25)
        todata = ToData()
        S = series(f, todata)
        S.reset()
        for x in data:
            S.put(x)
        S.end_input()
        res = todata.get_data()
        self.assertEqual(res, out)


