from abc import abstractmethod, ABCMeta

class FilterInterface:
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def process(self, data):
        ''' Processes the data in some way and returns
            an array of the same length. '''
        pass
