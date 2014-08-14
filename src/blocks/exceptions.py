import Queue

__all__ = [
   'Full',
   'NotReady',
   'Finished',
   'NeedInput',
   'NeedComputation',
   'Exhausted',
]

class Full(Queue.Full):
    pass


class Exhausted(Queue.Full):
    pass
    
class NotReady(Queue.Empty):
    pass

class Finished(Exception):
    pass


class NeedInput(Queue.Empty):
    """ Thrown when get() with block=True but there is no output available
        and what is needed is to get some input(). """
    pass

class NeedComputation(Queue.Empty):
    """ Thrown when get() says that a bit more computation is needed.
        Jusrt try get() again if you can spare the cycle. """
    pass
