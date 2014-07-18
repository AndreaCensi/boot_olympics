from .exceptions import Finished, Full, NeedComputation, NeedInput, NotReady
from .interface import SimpleBlackBox, Sink, Source
from contracts import contract
from types import NoneType
import time
import warnings




__all__ = [
   'bb_pump',
   'bb_pump_block',
   'bb_get_block_poll_sleep',
   'bb_pump_block_yields',
]

@contract(a=Source, b=Sink)
def bb_pump(a, b):
    """ Pumps from a to b until it a is not ready or b is Full; 
        returns number of iterations.
        
        :raise: Finished """
    num = 0
    while True:
        try:
            # print('%s: reading' % num)
            x = a.get(block=False)
            # print('%s: read %s' % (num, describe_value(x)))
        except NotReady:
            # print('not ready')
            break

        try:
            b.put(x, block=False, timeout=None)
        except Full:
            warnings.warn('Not sure what to do')
            break
        num += 1
    return num

@contract(a=Source, b=Sink)
def bb_pump_block(a, b):
    """ Pumps from a to b until it is finished; returns number of iterations.
        Raise NeedInput """
    num = 0
    while True:
        try:
            # print('%s: reading' % num)
            x = a.get(block=True)
            # print('%s: read %s' % (num, describe_value(x)))
        except NotReady:
            continue
        except Finished:
            break
        b.put(x, block=True)
        num += 1
    return num


@contract(a=Source, b=Sink)
def bb_pump_block_yields(a, b):
    """ Pumps from a to b until it is finished; yields each object. """
    while True:
        try:
            # print('%s: reading' % num)
            x = a.get(block=True)
            # print('%s: read %s' % (num, describe_value(x)))
        except  NotReady:
            continue
        except  Finished:
            break
        yield x
        b.put(x, block=True)

def bb_get_block_poll_sleep(bb, timeout, sleep):
    t0 = time.time()
    while True:
        t1 = time.time()
        delta = t1 - t0
        if timeout is not None and delta > timeout:
            msg = 'bb_get_block_poll_sleep: timeout: %s > %s' % (delta, timeout)
            raise  NotReady(msg)
        try:
            value = bb.get(block=False)
            assert not isinstance(value, NoneType)
            return value
        except  NotReady as e:
            bb.info('bb_get_block_poll_sleep: not ready, waiting: %s' % e)
            pass

        time.sleep(sleep)


@contract(source=Source, returns='list(tuple(float, *))')
def source_read_all_block(source):
    """ Reads all data from the source until it is finished,
        calling with blocking. """
    res = []
    while True:
        try:
            value = source.get(block=True)
            res.append(value)
        except Finished:
            break
        except NeedComputation:
            continue
        except NeedInput:
            assert isinstance(source, SimpleBlackBox)
            msg = 'source_read_all_blocks() can only read from a simple Source.'
            raise ValueError(msg)
            continue
    return res


@contract(input_sequence='list', bb=SimpleBlackBox, returns='list')
def bb_simple_interaction_blocking(bb, input_sequence):
    """ 
        Resets the SimpleBlackBox, puts the input sequence, returns the result. 
        Everything is blocking. 
    """ 
    assert isinstance(bb, SimpleBlackBox)
    bb.reset()
    
    # we are going to write this sequence
    obs = list(input_sequence)

    # and this collects the output
    out = []
    
    while True:
        # Put something -- always succeeds
        if obs:
            ob = obs.pop(0)
            bb.put(ob, block=True)
        else:
            # at the end, call obs.pop()
            bb.end_input()
        
        # Get all the output
        try: 
            while True:
                try:
                    o = bb.get(block=True)
                    # Do something with o
                    out.append(o)
                except Finished:
                    raise
                except NotReady:
                    assert(False) # block is True
                except NeedComputation:
                    continue
        except Finished:
            # probably finished only after we called end_input()
            assert not obs
            break

    return out


@contract(input_sequence='list', bb=SimpleBlackBox, returns='list')
def bb_simple_interaction_nonblocking(bb, input_sequence, timeout, timewait):
    """ 
        Resets the SimpleBlackBox, puts the input sequence, returns the result. 
        Everything is nonblocking.
        
        :param timeout: Timeout for calling get().
        :param timewait: How long to wait before re-calling get() if timeout.
         
    """ 
    assert isinstance(bb, SimpleBlackBox)
    bb.reset()
    
    # we are going to write this sequence
    obs = list(input_sequence)

    # and this collects the output
    out = []
    
    def wait_a_little():
        time.sleep(timewait)
            
    while True:
        if obs:
            ob = obs.pop(0)
            try:
                bb.put(ob, block=False, timeout=timeout)
            except Full:
                wait_a_little()
                continue
        else:
            # at the end, call obs.pop()
            bb.end_input()
        
        # Get all the output
        try: 
            while True:
                try:
                    o = bb.get(block=False, timeout=timeout)
                    # Do something with o
                    out.append(o)
                except Finished:
                    raise
                except NotReady:
                    wait_a_little()
                    continue
                except NeedComputation:
                    continue
        except Finished:
            # probably finished only after we called end_input()
            assert not obs
            break
    return out






