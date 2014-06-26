from blocks import WithQueue

__all__ = ['Identity']
    
class Identity(WithQueue):

    def put(self, value, block, timeout):  # @UnusedVariable
        self.append(value)
        
