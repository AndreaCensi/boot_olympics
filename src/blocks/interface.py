from abc import abstractmethod

from contracts import ContractsMeta, contract

from decent_logs import WithInternalLog


__all__ = [
    'Sink',
    'Source',
    'SimpleBlackBox',
]


class Sink(WithInternalLog):
    __metaclass__ = ContractsMeta

    @abstractmethod
    def reset(self):
        """ Resets the sink.   You must call reset() at least once before put()."""

    @abstractmethod
    @contract(block='bool', value='*', timeout='None|>=0')
    def put(self, value, block=True, timeout=None):
        """ 
            Raises Full if the object cannot take it
            and block is False.
            
            :raise: Full
        """

    def end_input(self):
        """ Signals the end of the input stream. """

    
class Source(WithInternalLog):
    __metaclass__ = ContractsMeta
    
    @abstractmethod
    def reset(self):
        """ Rewinds the source. You must call reset() at least once before get(). """
 

    @abstractmethod
    @contract(block='bool', timeout='None|>=0', returns='*')
    def get(self, block=True, timeout=None):
        """
            Same semantics of arguments for ``Queue.get``.
                timeout=None: blocking -> raises NeedComputation, Finished
                timeout=0 nonblock -> raises NotReady, NeedComputation, Finished

            
            If this is also a ``SimpleBlackBox``, the ``get()`` method can raise 
            NeedInput, meaning that
            you need to put more input in the black box to obtain an output.
                
            :raise NotReady: needs to wait a bit of wall-time before getting the next
            :raise NeedComputation: call again to trigger more computation 
            :raise Finished: finished: no more output will come
            
        """
    


class SimpleBlackBox(Sink, Source):
    """ 
        A generic interface for event-based dynamical systems. 
        
        In addition to the exceptions defined in the Sink interface,
        the ``get()`` method can raise NeedInput, meaning that
        you need to put more input in the black box to obtain an output.
        
        Here are a couple of examples for interacting with a generic BB.
        See also: ``bb_simple_interaction_nonblocking``, ``bb_simple_interaction_blocking``.
        
        Sample interaction with blocking behavior: ::
        
            m = ...
            assert isinstance(m, SimpleBlackBox)
            m.reset()
            
            # we are going to write this sequence
            obs = [1,2,3,4,5]
            # and this collects the output
            out = []
            
            while True:
                # Put something -- always succeeds
                if obs:
                    ob = obs.pop(0)
                    m.put(ob, block=True)
                else:
                    # at the end, call obs.pop()
                    m.end_input()
                
                # Get all the output
                try: 
                    while True:
                        try:
                            o = m.get(block=True)
                            # Do something with o
                            out.append(o)
                        except Finished:
                            raise
                        except NotReady:
                            assert(False) # block is True
                        except NeedComputation:
                            continue
                except Finished:
                    # probably finished only after we called end_input()
                    assert not obs
                    break
        
        Sample interaction with non blocking behavior (i.e. NotReady): ::

            m = ...
            assert isinstance(m, SimpleBlackBox)
            m.reset()
            
            # we are going to write this sequence
            obs = [1,2,3,4,5]
            # and this collects the output
            out = []
                        
            timeout=0.01
            timewait=0.1
            
            def wait_a_little():
                time.sleep(timewait)
            
            while True:
                if obs:
                    ob = obs.pop(0)
                    try:
                        m.put(ob, block=False, timeout=timeout)
                    except Full:
                        wait_a_little()
                        continue
                else:
                    # at the end, call obs.pop()
                    m.end_input()
                
                # Get all the output
                try: 
                    while True:
                        try:
                            o = m.get(block=False, timeout=timeout)
                            # Do something with o
                            out.append(o)
                        except Finished:
                            raise
                        except NotReady:
                            wait_a_little()
                            continue
                        except NeedComputation:
                            continue
                except Finished:
                    # probably finished only after we called end_input()
                    assert not obs
                    break
                    
    """


