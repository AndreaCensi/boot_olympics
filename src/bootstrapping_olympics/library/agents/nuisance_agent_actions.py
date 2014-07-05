from contracts import contract

from blocks import Sink
from blocks.composition import series
from blocks.library import Collect, Identity, Route, WithQueue
from bootstrapping_olympics import RepresentationNuisanceCausal


__all__ = [
    'wrap_agent_learner',
]

@contract(learner=Sink, rnc=RepresentationNuisanceCausal, returns=Sink)
def wrap_agent_learner(learner, rnc):
    # G = transform.get_G()
    Gc = rnc.get_G_conj()
    H = rnc.get_H()
    #
    # bd -> |expand| -> commands, observations
    #
    # cmd --> |G*| --> cmd' ---------------> |collect| -> learner
    #                   |                |
    #                   v                |
    # obs -----------> |H| ----obs'-------

    r1 = Route([({'commands':'commands'}, Gc, {'commands':'commands'}),
          ({'observations':'observations'}, Identity(), {'observations':'observations'})])

    r2 = Route([({'observations':'observations',
            'commands':'commands'}, H, {'observations':'observations'}),
          ({'commands':'commands'}, Identity(), {'commands': 'commands'})])

    sys = series(BootExpand(), r1, r2, Collect(), BootPutTimestamp(), learner)
    return sys

class BootExpand(WithQueue):
    def put_noblock(self, value):
        t, bd = value
        self.append((t, ('commands', bd['commands'])))
        self.append((t, ('observations', bd['observations'])))

class BootPutTimestamp(WithQueue):
    def put_noblock(self, value):
        t, bd = value
        bd['timestamp'] = t
        self.append((t, bd))

