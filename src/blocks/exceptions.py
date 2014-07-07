import Queue

__all__ = [
   'Full',
   'NotReady',
   'Finished',
]

class Full(Queue.Full):
    pass

class NotReady(Queue.Empty):
    pass

class Finished(Exception):
    pass


class NeedInput(Queue.Empty):
    """ Thrown when get() with block=True but there is no output available
        and what is needed is to get some input(). """
    pass

