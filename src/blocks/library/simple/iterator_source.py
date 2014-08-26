from abc import abstractmethod
from blocks import Finished, Source
from contracts import contract, describe_type, indent
from types import GeneratorType
import string
import traceback

__all__ = [
    'IteratorSource',
]


class IteratorSource(Source):

    def reset(self):
        self.iterator = self.get_iterator()

    @abstractmethod
    @contract(returns=GeneratorType)
    def get_iterator(self):
        """ Returns iterator to use. """
        pass

    def get(self, block=True, timeout=None):  # @UnusedVariable
        try:
            res = self.iterator.next()
            return res
        except StopIteration:
            raise Finished
        except Exception as e:
            msg = 'Could not call next() on user-given iterator.\n'
            msg += '   iterator: %s\n' % str(self.iterator)
            msg += '    of type: %s\n' % describe_type(self.iterator)
            msg += 'because of this error:\n'
            msg += indent(string.strip('%s\n%s' % (e, traceback.format_exc(e))), '| ')
            self.error(msg)
            raise
