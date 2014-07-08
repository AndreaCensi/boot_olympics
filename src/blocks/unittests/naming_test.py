from contracts import contract

from blocks.composition import series, series_multi, series_two
from blocks.library import (Delay, Identity, NameSignal, Split,
                            WithQueue, Collect, Route)
from blocks.utils import check_reset

from .blocks_testing_utils import BlocksTest


class HTest(WithQueue):

    def __init__(self):
        WithQueue.__init__(self)

    def reset(self):
        WithQueue.reset(self)
        self.sign = +1

    def put_noblock(self, value):
        check_reset(self, 'sign')

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


class NamingTests(BlocksTest):


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

        series = series_two

        split = Split(NameSignal('commands'),
                      series(G, series(R, NameSignal('observations'), 'R', 'ns_obs'),
                            'G', 'Rns_obs'),
                            'ns_cmd', 'GRn')

        bbox = series(split, H, 'split', 'H')

        data = [(0.0, 0), (1.0, 1), (2.0, 2), (3.0, 3)]
        expected = [(0.0 + d, 0),
                    (1.0 + d, -1), (2.0 + d, 2), (3.0 + d, -3)]
        self.check_bbox_results(bbox, data, expected)


    def complex_nuisance_test_2(self):
        R = Identity()
        G = Identity()
        G2 = Identity()
        bbox = series(G, series(R, G2))

        data = [(0.0, 10), (1.0, 20), (2.0, 31)]
        expected = data
        self.check_bbox_results(bbox, data, expected)



    def complex_nuisance_test_3(self):

        Gc = Identity()
        H = series(HTest(), NameSignal('observations'))

        # note: woudl not work with observations arriving vefore commands
        # because we memorize the last command given
        data = [
            (-1.0, ('commands', 1)),
            (-1.0, ('observations', -10)),
            (1.0, ('commands', 2)),
            (1.0, ('observations', 10)),
            (2.0, ('commands', 1)),
            (2.0, ('observations', 20)),
            (3.0, ('commands', 2)),
            (3.0, ('observations', 30)),
         ]
        #
        # bd -> |expand| -> commands, observations
        #
        # cmd --> |G*| --> cmd' ---------------> |collect| -> learner
        #                   |                |
        #                   v                |
        # obs -----------> |H| ----obs'-------

        r1 = Route([({'commands':'commands'}, Gc, {'commands':'commands'}),
              ({'observations':'observations'}, Identity(),
               {'observations':'observations'})])

        r2 = Route([({'observations':'observations',
                'commands':'commands'}, H, {'observations':'observations'}),
              ({'commands':'commands'}, Identity(), {'commands': 'commands'})])

        expected = [
            (-1.0, dict(observations=10, commands=1)),
            (1.0, dict(observations=10, commands=2)),
            (2.0, dict(observations=-20, commands=1)),
            (3.0, dict(observations=30, commands=2)),
        ]

        bbox = series_multi(r1, r2, Collect())
        self.check_bbox_results(bbox, data, expected)






