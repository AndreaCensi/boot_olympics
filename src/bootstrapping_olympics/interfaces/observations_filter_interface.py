from abc import abstractmethod

class FilterInterface:
    
    @abstractmethod
    def process(self, data):
        ''' Processes the data in some way and returns
            an array of the same length. '''
        pass
