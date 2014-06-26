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
