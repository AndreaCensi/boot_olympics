from contracts import describe_value

from .with_queue import WithQueue


__all__ = ['Info']

class Info(WithQueue):

    def put_noblock(self, value):
        t, ob = value
        self.info('obtained %s %s' % (t, describe_value(ob)))
        self.append(value)

