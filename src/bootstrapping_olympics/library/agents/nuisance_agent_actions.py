from blocks import   SimpleBlackBox, Sink, Source
from blocks.composition import series
from blocks.library import Identity, Route, WrapTMfromT
from bootstrapping_olympics import (RepresentationNuisance, 
    RepresentationNuisanceCausal, get_conftools_nuisances, 
    get_conftools_nuisances_causal)
from contracts import contract
from contracts.utils import check_isinstance
from blocks import SimpleBlackBoxT, SimpleBlackBoxTN
from blocks.library.loops import loopit



__all__ = [
    'wrap_agent_learner',
    'wrap_agent_explorer',
    'wrap_agent_servo',
]



@contract(explorer=SimpleBlackBox,
          rnc=RepresentationNuisanceCausal,
          returns=SimpleBlackBox)
def wrap_agent_explorer(explorer, rnc):
    #        -----<---
    #       /         \
    # y -> |L| -> |A| -|- > |G| -> u
    G = rnc.get_G()
    assert isinstance(G, SimpleBlackBoxT) and not isinstance(G, SimpleBlackBoxTN)
    L = rnc.get_L()
    
    LA = series(L, explorer)
    # LA has input "observations" and "commands" and output "commands"
    # Loop it on itself 
    
    loopLA = loopit(LA, 'commands', 'commands')
    res = series(loopLA, WrapTMfromT(G))
    res.set_names(['loopLA','G'])
    return res

@contract(servo=SimpleBlackBox,
          rnc=RepresentationNuisanceCausal,
          returns=SimpleBlackBox)
def wrap_agent_servo(servo, rnc):
    # servo takes "observations" and "goal_observations"
    
    #         -----<---
    #        /         \
    # y  -> |L| -\      |
    #              |A| -|- > |G| -> u
    # yg -> |L| -/      |
    #        `---------/
    
    G = rnc.get_G()
    Ly = rnc.get_L()
    Lyg = rnc.get_L()
    
    LL = Route([
        ({'observations':'observations', 'commands':'commands'}, Ly, {'observations':'observations'}),
        ({'goal_observations':'observations', 'commands':'commands'}, Lyg, {'observations':'goal_observations'}),
    ])
    LL.set_names(['Ly', 'Lyg'])
    LLA = series(LL, servo)
    LL.set_names(['LL', 'servo'])
    # LLA has input "observations", "goal_observations" and "commands" and output "commands"
    # Loop it on itself 
    loopLLA = loopit(LLA, 'commands', 'commands')
    res = series(loopLLA, WrapTMfromT(G))
    res.set_names(['loopLLA','G'])
    return res

 
@contract(learner=Sink, rnc=RepresentationNuisanceCausal, returns=Sink)
def wrap_agent_learner(learner, rnc):
    # 
    #  u -> G* --+-----> u'  
    #            V          => learner
    #  y -----> |L| ---> y'
    
    Gc = rnc.get_G_conj()
    L = rnc.get_L()
    
    r1 = Route([({'commands':'commands'}, WrapTMfromT(Gc), {'commands':'commands'}),
          ({'observations':'observations'}, Identity(), {'observations':'observations'})])
    r1.set_names(['Gc', 'Id'])

    r2 = Route([({'observations':'observations',
            'commands':'commands'}, L, {'observations':'observations'}),
          ({'commands':'commands'}, Identity(), {'commands': 'commands'})])
    r2.set_names(['L', 'Id'])
    
    res = series(r1, r2, learner)
    res.set_names(['r1','r2','learner'])
    return res


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


@contract(R=SimpleBlackBox, nuisance=RepresentationNuisanceCausal,
          returns=SimpleBlackBox)
def wrap_robot_exploration(R, nuisance):
    """ Does not returns not a named signal system. """
    G = nuisance.get_G()
    L = nuisance.get_L()
    
    #   u' -> |G| -> u- > |R| -> y -> |L| -> y'
    #    \-----------------------------/

    GR = series(WrapTMfromT(G, 'G'), R)
    GR.set_names(['TMfromT','R'])

    r1 = Route([({'commands':'commands'}, GR, {'observations':'obs1'}),
                ({'commands':'commands'}, Identity(), {'commands':'commands'})])
                
    r2 = Route([({'obs1':'observations', 'commands':'commands'}, 
                L, 
                {'observations':'observations'})])
    r2.set_names(['L'])
    res = series(r1, r2)
    res.set_names(['r1','r2'])
    return res



@contract(source=Source, nuisance=RepresentationNuisanceCausal,
          returns=Source)
def wrap_robot_passive_stream(source, nuisance):
    raise NotImplementedError()
        




# class BootExpand(WithQueue):
# 
#     @contract(value='tuple(float, *)')
#     def put_noblock(self, value):
#         if not isinstance(value, tuple) or len(value) != 2:
#             msg = 'Expected a tuple'
#             raise ValueError(msg)
# 
#         t, bd = value
# #         self.info('expanding bd in "commands" and "observations"')
#         self.append((t, ('commands', bd['commands'])))
#         self.append((t, ('observations', bd['observations'])))
# 
#     def __str__(self):
#         return 'BootExpand()'
# 
# class BootPutTimestamp(WithQueue):
#     @contract(ignore_incomplete='bool')
#     def __init__(self, ignore_incomplete):
#         """
#             :param ignore_incomplete: just ignore packets that don't have
#              both commands and observations
#         """
#         self.ignore_incomplete = ignore_incomplete
#         WithQueue.__init__(self)
# 
#     @contract(value='tuple(float, *)')
#     def put_noblock(self, value):
#         if not isinstance(value, tuple) or len(value) != 2:
#             msg = 'Expected a tuple'
#             raise ValueError(msg)
# 
#         t, bd = value
#         if (not 'observations' in bd) or (not 'commands' in bd):
#             msg = 'BootPutTimestamp: Expected obs/cmd fields in bd: %s' % bd
#             if self.ignore_incomplete:
#                 self.warn(msg)
#                 return
#             else:
#                 raise ValueError(msg)
#         bd['timestamp'] = t
#         self.append((t, bd))
# 
#     def __str__(self):
#         return 'BootPutTimestamp()'

# 
# class CheckBootSpec(Instantaneous):
#     """ Makes sure that the spec is respected. """
# 
#     def __str__(self):
#         return 'CheckBootSpec()'
# 
#     def __init__(self, boot_spec):
#         Instantaneous.__init__(self)
#         self.boot_spec = boot_spec
#         self.obs_spec = boot_spec.get_observations()
#         self.cmd_spec = boot_spec.get_commands()
# 
#     def transform_value(self, value):
#         self.obs_spec.check_valid_value(value['observations'])
#         self.cmd_spec.check_valid_value(value['commands'])
#         return value
