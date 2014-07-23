from blocks.library import Identity

from .blocks_testing_utils import BlocksTest
from blocks.library.simple.with_queue import WithQueue
from blocks.composition import series
from blocks.exceptions import NeedInput
from blocks.library.timed_named.route import Route


class Prepared(WithQueue):
    """ Already has some in queue. """
    def __init__(self, some):
        self.some = some
        
    def reset(self):
        WithQueue.reset(self)
        for s in self.some:
            self.append(s)
            
    def put_noblock(self, value):
        # behaves as identity
        self.append(value)
    
class ReadyTest(BlocksTest):

    def ready_test1(self):
        prepared = Prepared([32,34,43])
        
        S = series(prepared, Identity())
        S.reset()
        assert 32 == S.get(block=True)
        assert 34 == S.get(block=True)
        assert 43 == S.get(block=True)
        self.assertRaises(NeedInput, S.get, block=True)
        S.put(3)
        assert 3 == S.get(block=True)
        
        
    def ready_test2(self):
        
        prepared = Prepared([32,34,43])
        
        # Now it's the other way around
        S = series(Identity(), prepared)
        S.reset()
    
        assert 32 == S.get(block=True)
        assert 34 == S.get(block=True)
        assert 43 == S.get(block=True)
        self.assertRaises(NeedInput, S.get, block=True)
        S.put(3)
        assert 3 == S.get(block=True)


    def ready_test3(self):
        
        prepared = Prepared([32,34,43])
        
        # Now it's the other way around
        S = series(Identity(), prepared, Identity())
        S.reset()
    
        assert 32 == S.get(block=True)
        assert 34 == S.get(block=True)
        assert 43 == S.get(block=True)
        self.assertRaises(NeedInput, S.get, block=True)
        S.put(3)
        assert 3 == S.get(block=True)
        
    def ready_test4_route(self):
        prepared_data = [
            (0.0, ('a', 'A0')),
            (1.0, ('a', 'A1')),
        ]
        
        routing = [
           # a is delayed
           ({'a':'a'}, Prepared(prepared_data), {'a':'a'}),
        ]

        bbox = Route(routing)
        bbox.reset()

        self.assertEqual(prepared_data[0], bbox.get(block=True))
        self.assertEqual(prepared_data[1], bbox.get(block=True))
        self.assertRaises(NeedInput, bbox.get, block=True)






