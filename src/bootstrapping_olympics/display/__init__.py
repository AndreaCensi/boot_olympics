try: 
    import reprep
    installed = True
except ImportError:
    import warnings
    warnings.warn('RepRep not installed; display not available.')
    installed = False

if installed:
    from .reprep_publisher import *
    
