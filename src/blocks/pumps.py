import warnings

from contracts import  contract

from blocks import SimpleBlackBox
from .simple_black_box import Source, Sink
import time
from blocks.exceptions import Finished


__all__ = [
   'bb_pump',
   'bb_pump_block',
   'bb_get_block_poll_sleep',
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
        except SimpleBlackBox.NotReady:
            # print('not ready')
            break

        try:
            b.put(x, block=False, timeout=None)
        except SimpleBlackBox.Full:
            warnings.warn('Not sure what to do')
            break
        num += 1
    return num

@contract(a=Source, b=Sink)
def bb_pump_block(a, b):
    """ Pumps from a to b until it is finished; returns number of iterations """
    num = 0
    while True:
        try:
            # print('%s: reading' % num)
            x = a.get(block=True)
            # print('%s: read %s' % (num, describe_value(x)))
        except SimpleBlackBox.NotReady:
            continue
        except SimpleBlackBox.Finished:
            break
        b.put(x, block=True)
        num += 1
    return num


def bb_get_block_poll_sleep(bb, timeout, sleep):
    t0 = time.time()
    while True:
        t1 = time.time()
        delta = t1 - t0
        if timeout is not None and delta > timeout:
            msg = 'timeout: %s > %s' % (delta, timeout)
            raise SimpleBlackBox.NotReady(msg)
        try:
            value = bb.get(block=False)
            return value
        except SimpleBlackBox.NotReady:
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
    return res
