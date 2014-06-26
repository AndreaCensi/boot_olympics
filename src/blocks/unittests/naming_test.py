from unittest.case import TestCase

from contracts import contract

from blocks.composition import series
from blocks.library import FromData
from blocks.library import Identity
from blocks.library import NameSignal
from blocks.library import Split
from blocks.pumps import source_read_all_block
from blocks.with_queue import WithQueue


class Delay(WithQueue):
    def __init__(self, delay):
        WithQueue.__init__(self)
        self.delay = delay

    def put(self, value, block=False, timeout=None):  # @UnusedVariable
        t, x = value
        t2 = t + self.delay
        self.append((t2, x))


class HTest(WithQueue):

    def __init__(self):
        WithQueue.__init__(self)
        self.sign = +1

    def put(self, value, block=False, timeout=None):  # @UnusedVariable
        if(not(isinstance(value, tuple) and len(value) == 2 and
            isinstance(value[1], tuple) and len(value[1]) == 2)):
            msg = 'Invalid value: %s' % value
            raise ValueError(msg)
        timestamp, (signal, x) = value
        self.put_signal(timestamp, signal, x)
    
    @contract(timestamp='float', signal='str', value='*')
    def put_signal(self, timestamp, signal, value):
        self.info('observed %10.4f %15s %10s' % (timestamp, signal, value))
        if signal == 'observations':
            value2 = value * self.sign
            self.append((timestamp, value2))
        elif signal == 'commands':
            print('observed commands (%s)' % value)
            self.sign = +1 if value % 2 == 0 else -1
        else:
            raise ValueError('No valid signal %r.' % signal)


class NamingTests(TestCase):


    def complex_nuisance_test_1(self):
        #   ...[as commands]........
        #  -> u -> |G| -> |R| -> |H| -> y
        #
        #
        print('Here we are given a system R and two nuisances G,H.')

        print('R \in D(Y;U)')
        print("G \in D(U';U)")
        print("H \in D(Y;Y,U')")

        d = 0.2
        R = Delay(d)
        G = Identity()
        H = HTest()

        split = Split(NameSignal('commands'),
                      series(G, series(R, NameSignal('observations'), 'R', 'ns_obs'),
                            'G', 'Rns_obs'),
                            'ns_cmd', 'GRn')

        S = series(split, H, 'split', 'H')

        data = [(0.0, 0), (1.0, 1), (2.0, 2), (3.0, 3)]
        expected = [(0.0 + d, 0), (1.0 + d, -1), (2.0 + d, 2), (3.0 + d, -3)]
        s = FromData(data)
        sys = series(s, S, 'fromdata', 'S')
        res = source_read_all_block(sys)
        print('the output is %s' % res)
        self.assertEqual(expected, res)

    def complex_nuisance_test_2(self):


        R = Identity()
        G = Identity()
        G2 = Identity()
#         H = HTest()

#         S = series(G, series(R, NameSignal('observations')))

        S = series(G, series(R, G2))
#         S = series(G, R)
#         
#         S = series(Split(NameSignal('commands'),
#                

        data = [(0.0, 10), (1.0, 20), (2.0, 31)]
        s = FromData(data)
        sys = series(s, S)
        res = source_read_all_block(sys)
        self.assertEqual(data, res)
