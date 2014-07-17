from blocks import SimpleBlackBox, Sink, Source
from blocks.composition import series
from blocks.library import (CollectSignals, Identity, Instantaneous, Route, 
    WithQueue)
from bootstrapping_olympics import (RepresentationNuisance, 
    RepresentationNuisanceCausal, get_conftools_nuisances, 
    get_conftools_nuisances_causal)
from contracts import contract
from contracts.utils import check_isinstance
import warnings



__all__ = [
    'wrap_agent_learner',
    'wrap_agent_explorer',
]


@contract(explorer=SimpleBlackBox,
          rnc=RepresentationNuisanceCausal,
          returns=SimpleBlackBox)
def wrap_agent_explorer(explorer, rnc):
    G = rnc.get_G()
    bd_transform = get_bd_transform(rnc, ignore_incomplete=True)
    sys = series(bd_transform, explorer, G)
    sys.set_names(['bd_transform', 'explorer', 'G'])

#     sys = series(bd_transform, explorer)  # raises NeedInput
#     sys = series(explorer, G)  # ok
    warnings.warn('xxx')
    return sys


    #
    # y        y'             u'        u
    # --> |H*| --> |explorer| ---> |G| -->
    #       \--u'---<-------__/

    # or, considering we want to do bd
    # bd -> |expand| -> commands, observations
    #
    # u -> |G*| -\ u'  y'             u'        u
    # --------> |H| --> |explorer| ---> |G| -->
#     # obs
#     Gc = rnc.get_G_conj()
#     G = rnc.get_G()
#
#     r1 = Route([({'commands':'commands'}, Gc, {'commands':'commands'}),
#           ({'observations':'observations'}, Identity(), {'observations':'observations'})])
#
#     H = rnc.get_H()
#
#     r2 = Route([({'observations':'observations',
#             'commands':'commands'}, H, {'observations':'observations'}),
#           ({'commands':'commands'}, Identity(), {'commands': 'commands'})])
#
#     ignore_incomplete = True
#     sys = series(BootExpand(), r1, r2,
#                  Collect(), BootPutTimestamp(ignore_incomplete),
#                  explorer, G)
#     return sys


@contract(learner=Sink, rnc=RepresentationNuisanceCausal, returns=Sink)
def wrap_agent_learner(learner, rnc):
    bd_transform = get_bd_transform(rnc, ignore_incomplete=True)
    return series(bd_transform, learner)


@contract(rnc=RepresentationNuisanceCausal, ignore_incomplete='bool',
          returns=SimpleBlackBox)
def get_bd_transform(rnc, ignore_incomplete):
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
    r1.set_names(['Gc', 'Id'])

    r2 = Route([({'observations':'observations',
            'commands':'commands'}, H, {'observations':'observations'}),
          ({'commands':'commands'}, Identity(), {'commands': 'commands'})])

    r2.set_names(['H', 'Id'])
    sys = series(BootExpand(), r1, r2,
                 CollectSignals(set(['observations', 'commands'])),
                 BootPutTimestamp(ignore_incomplete))

    sys.set_names(['BootExpand', 'r1', 'r2', 'CollectS', 'BootPutTs'])
    return sys


class BootExpand(WithQueue):

    @contract(value='tuple(float, *)')
    def put_noblock(self, value):
        if not isinstance(value, tuple) or len(value) != 2:
            msg = 'Expected a tuple'
            raise ValueError(msg)

        t, bd = value
#         self.info('expanding bd in "commands" and "observations"')
        self.append((t, ('commands', bd['commands'])))
        self.append((t, ('observations', bd['observations'])))

    def __str__(self):
        return 'BootExpand()'

class BootPutTimestamp(WithQueue):
    @contract(ignore_incomplete='bool')
    def __init__(self, ignore_incomplete):
        """
            :param ignore_incomplete: just ignore packets that don't have
             both commands and observations
        """
        self.ignore_incomplete = ignore_incomplete
        WithQueue.__init__(self)

    @contract(value='tuple(float, *)')
    def put_noblock(self, value):
        if not isinstance(value, tuple) or len(value) != 2:
            msg = 'Expected a tuple'
            raise ValueError(msg)

        t, bd = value
        if (not 'observations' in bd) or (not 'commands' in bd):
            msg = 'BootPutTimestamp: Expected obs/cmd fields in bd: %s' % bd
            if self.ignore_incomplete:
                self.warn(msg)
                return
            else:
                raise ValueError(msg)
        bd['timestamp'] = t
        self.append((t, bd))

    def __str__(self):
        return 'BootPutTimestamp()'


class CheckBootSpec(Instantaneous):
    """ Makes sure that the spec is respected. """

    def __str__(self):
        return 'CheckBootSpec()'

    def __init__(self, boot_spec):
        Instantaneous.__init__(self)
        self.boot_spec = boot_spec
        self.obs_spec = boot_spec.get_observations()
        self.cmd_spec = boot_spec.get_commands()

    def transform_value(self, value):
        self.obs_spec.check_valid_value(value['observations'])
        self.cmd_spec.check_valid_value(value['commands'])
        return value

@contract(nuisances='list(str|code_spec|isinstance(RepresentationNuisance)'
                          '|isinstance(RepresentationNuisanceCausal))',
        returns=RepresentationNuisanceCausal)
def instance_nuisance_series(nuisances):
    config_nuisances_causal = get_conftools_nuisances_causal()
    config_nuisances = get_conftools_nuisances()
    ns = []
    for n in nuisances:

        if isinstance(n, str):
            c1 = n in config_nuisances
            c2 = n in config_nuisances_causal
            num = (1 if c1 else 0) + (1 if c2 else 0)
            if num == 0:
                msg = 'Could not find %r as either type of nuisance.' % n
                raise ValueError(msg)
            if num == 2:
                msg = 'Ambiguous name %r.' % n
                raise ValueError(msg)
            if c1:
                assert not c2
                _, x = config_nuisances.instance_smarter(n)
            else:
                _, x = config_nuisances_causal.instance_smarter(n)
                assert c2
        else:
            try:
                _, x = config_nuisances_causal.instance_smarter(n)
            except:
                _, x = config_nuisances.instance_smarter(n)

        if isinstance(x, RepresentationNuisance):
            from bootstrapping_olympics.library.nuisances_causal import SimpleRNCObs
            x = SimpleRNCObs(x)

        check_isinstance(x, RepresentationNuisanceCausal)
        ns.append(x)

    from bootstrapping_olympics.library.nuisances_causal import series_rnc
    nuisance = series_rnc(*tuple(ns))
    return nuisance


@contract(bb=SimpleBlackBox, nuisance=RepresentationNuisanceCausal,
          returns=SimpleBlackBox)
def wrap_robot_exploration(bb, nuisance):
    G = nuisance.get_G()
    bdt = get_bd_transform(nuisance, ignore_incomplete=True)
    s = series(G, bb, bdt)
    s.set_names(['G', 'R', 'bdt'])
    return s

@contract(source=Source, nuisance=RepresentationNuisanceCausal,
          returns=Source)
def wrap_robot_passive_stream(source, nuisance):
    raise NotImplementedError()
        
