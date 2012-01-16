from . import contract
import time


class InAWhile:

    @contract(interval='None|>0')
    def __init__(self, interval=5):
        self.count = 0
        self.start = time.time()
        self.interval = interval
        self.last = self.start
        self.now = self.start

    def its_time(self):
        self.count += 1
        self.now = time.time()

        if self.interval is None:
            return False

        if self.now >= self.last + self.interval:
            self.last = self.now
            return True
        else:
            return False

    def fps(self):
        if self.count == 0:
            return 0
        return self.count / (self.now - self.start)
