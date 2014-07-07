from abc import abstractmethod

from contracts import ContractsMeta, contract

from decent_logs import WithInternalLog

from .exceptions import Finished, NotReady, Full


__all__ = [
    'Sink',
    'Source',
    'SimpleBlackBox',
]


class Sink(WithInternalLog):
    __metaclass__ = ContractsMeta

    Full = Full

    @abstractmethod
    def reset(self):
        """ Resets the sink. """
        pass

    @abstractmethod
    @contract(block='bool', value='*', timeout='None|>=0')
    def put(self, value, block=False, timeout=None):
        """ 
            Raises Full if the object cannot take it
            and block is False.
            
            :raise: Full
        """

    def end_input(self):
        """ Signals the end of the input stream. """
        pass

    
class Source(WithInternalLog):
    __metaclass__ = ContractsMeta

    NotReady = NotReady
    Finished = Finished

    @abstractmethod
    def reset(self):
        """ Rewinds the source. """

    # def ready():
    # def finished():

    @abstractmethod
    @contract(block='bool', timeout='None|>=0', returns='*')
    def get(self, block=True, timeout=None):
        """
            Same semantics of arguments for ``Queue.get``.
            
            timeout=None: blocking
            timeout=0 nonblock
            
            :raise: NotReady
            :raise: Finished
            
        """
        pass
    


class SimpleBlackBox(Sink, Source):
    """ 
        A first try to make a generic interface for dynamical systems. 
        
       
        The get() method can raise NeedInput.
        
        
        Interaction: ::
        
            while True:
                m = Block()
                m.write(1)
                #while m.update() == m.UPDATE_NOT_FINISHED:
                #    pass
                try:
                    while True:
                        try:
                            o = m.read(timeout=0)
                            yield o
                        except NotReady:
                            break
                except Finished:
                    break
    """


