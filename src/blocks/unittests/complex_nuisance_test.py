from unittest.case import TestCase

from blocks.composition import series
from blocks.library.from_data import FromData
from blocks.library.identity import Identity
from blocks.pumps import source_read_all_block
from blocks.with_queue import WithQueue
import numpy as np


class SuperSample(WithQueue):

    def __init__(self, min_interval):
        """ This guarantees that the output
            is at most min_interval. """
        WithQueue.__init__(self)
        self.min_interval = min_interval
        self.last = None

    def put(self, value, block=False, timeout=None):
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
        if self.last is not None:
            self.append(self.last)
        WithQueue.end_input(self)




class ConnectionTests(TestCase):

    def from_data_test1(self):
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        s = FromData(data)
        res = source_read_all_block(s)
        self.assertEqual(data, res)

    def series_test_1(self):
        """  Series(FromDatA, Identity) """
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        s = FromData(data)
        f = Identity()
        S = series(s, f)
        res = source_read_all_block(S)
        self.assertEqual(data, res)

    def series_test_2(self):
        """  Series(FromDatA, SuperSample) """
        data = [(0.0, 'A'), (1.0, 'B'), (2.0, 'C')]
        out = [(0.0, 'A'), (0.25, 'A'), (0.5, 'A'), (0.75, 'A'),
               (1.0, 'B'), (1.25, 'B'), (1.5, 'B'), (1.75, 'B'),
               (2.0, 'C')]
        f = SuperSample(0.25)
        S = series(FromData(data), f)
        res = source_read_all_block(S)
        self.assertEqual(res, out)


