from contracts import contract

class Publisher:
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''
    
    @contract(name='str', value='array')
    def publish_array(self, name, value):
        ''' Publishes an array; the publisher decides how to visualize it. '''
    
    FILTER_POSNEG = 'posneg'
    FILTER_SCALE = 'scale'
    @contract(name='str', value='array')
    def publish_array_as_image(self, name, value, filter='posneg', filter_params={}):
        ''' 
            Publishes an array as a false-color image. 
            ``filter`` is the name of a filter.
            ``filter_params`` are parameters to pass to the filter.
        '''
    
    @contract(name='str', text='str')
    def publish_text(self, name, text):
        ''' 
            Publishes a text object.

            Example: ::
            
                p.publish_text('status', 'I am ok')
        '''
    
