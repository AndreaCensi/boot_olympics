from abc import abstractmethod
from contracts.metaclass import ContractsMeta
from contracts import contract

__all__ = ['SimpleBlackBox']


class SimpleBlackBox(object):
    """ 
        A first try to make a generic interface for dynamical systems. 
        
        
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
    
    __metaclass__ = ContractsMeta
    
    @abstractmethod
    def put(self, value):
        pass

    class NotReady(Exception):
        pass 
    
    class Finished(Exception):
        pass 
    
    @abstractmethod
    @contract(block='bool', timeout='None|>=0', returns='tuple')
    def get(self, block=True, timeout=None):
        """
            Same semantics of arguments for ``Queue.get``.
            
            timeout=None: 
            timeout=0 nonblock
            
        """
        pass
