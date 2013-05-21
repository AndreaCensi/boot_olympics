from typsy_dynamics.bb import BlackBox
from typsy.unittests.test_inference import typsy_type

class DynamicalSystem():
    # TODO: change interface
    
    def get_typsy_type(self):
        pass
    
    def write(self, value):
        pass

    class NotReady(Exception):
        pass 
    
    class Finished(Exception):
        pass 
    
    def read(self, timeout=None):
        """
            timeout=None: 
            timeout=0 nonblock
        """
        pass

    def next_data_status(self):
        ''' 
        This is complicated but necessary. Do you have another value?
        
        - No, this generator has finished.

        - Yes, and it will be of this timestamp. (I can see it from the log)
                
        - Yes, but this is realtime and it does not depend on me.
          For example, I'm waiting for the next sensor data. 
          Ask me later. 
          
        In those cases, we return:
        
            (True, timestamp)
            
            (False, None)
            
            (True, None)
          
        '''
        raise NotImplementedError('"next_data_status" was not implement.')


@typsy_type("BB(B;A) x BB(C;B) -> BB(C;A)")
class Composition(DynamicalSystem):
    
    def __init__(self, a, b):
        self.a = a
        self.b = b
    
    def get_typsy_type(self):
        ta = self.a.get_typsy_type()
        tb = self.b.get_typsy_type()
        return BlackBox(ta.i, tb.o, ta.t)
    
    def read(self, timeout=None):
        return self.b.read(timeout)
    
    def write(self, value):
        self.a.write(value)
        while True:
            try:
                r = self.a.read(timeout=0)
            except DynamicalSystem.NotReady:
                break
            self.b.write(r)
            

@typsy_type("BB(B;A) x BB(C;A) -> BB(B x C;A)")
class Series(DynamicalSystem):
    def __init__(self, a, b):
        self.a = a
        self.b = b
    
    def read(self, timeout=None):
        return self.b.read(timeout)
    
    def write(self, value):
        self.a.write(value)
        while True:
            try:
                r = self.a.read(timeout=0)
            except DynamicalSystem.NotReady:
                break
            self.b.write(r)
            
            
    
    
    
    
