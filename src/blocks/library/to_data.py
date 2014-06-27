from blocks import Sink


__all__ = ['ToData']


class ToData(Sink):
    """ A simple Sink that collects all data sent to it. """

    def __init__(self):
        self.data =[]
        
    def put(self, value, block, timeout):
        self.data.append(value)

    def get_data(self):
        return self.data
