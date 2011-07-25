import os
import datetime
import time

def expand_environment(s):
    ''' Expands ~ and ${ENV} in the string. '''
    s = os.path.expandvars(s)
    s = os.path.expanduser(s)
    return s

def isodate():
    now = datetime.datetime.now()
    date = now.isoformat('-')[:16]
    return date


class InAWhile:
    
    def __init__(self, interval=5):
        self.count = 0
        self.start = time.time()
        self.interval = interval
        self.last = self.start
        self.now = self.start
        
    def its_time(self):
        self.count += 1
        self.now = time.time()
        
        if self.now >= self.last + self.interval:
            self.last = self.now
            return True
        else:
            return False
    
    def fps(self):
        if self.count == 0:
            return 0
        return self.count / (self.now - self.start) 
