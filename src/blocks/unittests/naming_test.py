from blocks.with_queue import WithQueue
from unittest.case import TestCase
from blocks.library.identity import Identity
import numpy as np
from contracts import contract
from blocks.library.from_data import FromData
from blocks.pumps import source_read_all_block
from blocks.composition import series
from blocks.simple_black_box import SimpleBlackBox
from blocks.exceptions import NotReady, Finished
from IPython.testing.decorators import skip
# 
# 
# class NamedSignal(WithQueue):
# 
#     def __init__(self):
#         WithQueue.__init__(self)
# 
#     def put(self, value, block=False, timeout=None):
#         if self.last is not None:
#             t1, v1 = self.last
#             t2, _ = value
#             if t2 - t1 > self.min_interval:
#                 n = np.ceil((t2 - t1) / self.min_interval)
#                 n = int(n)
#                 for i in range(n):
#                     t = t1 + (i) * self.min_interval
#                     v = v1
#                     self.append((t, v))
#         self.last = value
# 
#     def end_input(self):
#         if self.last is not None:
#             self.append(self.last)
#         WithQueue.end_input(self)



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
        if signal == 'observations':
            self.append((timestamp, value * self.sign))
        elif signal == 'commands':
            self.sign = np.sign(value)
        else:
            raise ValueError('No valid signal %r.' % signal)
           
class NameSignal(WithQueue): 

    def __init__(self, name):
        WithQueue.__init__(self)
        self.name = name

    def put(self, value, block=False, timeout=None):
        timestamp, ob = value
        x = timestamp, (self.name, ob)
        self.append(x)

class Split(WithQueue):
    
    @contract(a=SimpleBlackBox, b=SimpleBlackBox)
    def __init__(self, a, b):
        WithQueue.__init__(self)
        self.a = a
        self.b = b
        self.a_finished = False
        self.b_finished = False

    def put(self, value, block=False, timeout=None):
        # TODO: does not consider Full as a special case
        self.a.put(value, block, timeout)
        self.b.put(value, block, timeout)
        new_obs = []
        if not self.a_finished:
            while True:
                try: 
                    x = self.a.get(block=False)
                    new_obs.append(x)
                except NotReady:
                    break
                except Finished:
                    self.a_finished = True
        if not self.b_finished:
            while True:
                try: 
                    x = self.b.get(block=False)
                    new_obs.append(x)
                except NotReady:
                    break
                except Finished:
                    self.b_finished = True
        # Sort by timestamp
        s = sorted(new_obs, key=lambda x: x[0])
#         print('sorted: %s' % s)
        for x in s:
            self.append(x)
        
        if self.a_finished and self.b_finished:
            self._finished = True
                
    def end_input(self):
        self.a.end_input()
        self.b.end_input()


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

        R = Identity()
        R.set_name_for_log('R')
        G = Identity()
        G.set_name_for_log('G')
        H = HTest()
        H.set_name_for_log('H')

        split = Split(NameSignal('commands'),
               series(G, series(R, NameSignal('observations'))))
        split.set_name_for_log('split')

        S = series(split, H)

        data = [(0.0, 10), (1.0, 20), (2.0, 31)]
        s = FromData(data)
        s.set_name_for_log('FromData')
        sys = series(s, S)
        sys.set_name_for_log('root')
        res = source_read_all_block(sys)
        self.assertEqual(data, res)



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
