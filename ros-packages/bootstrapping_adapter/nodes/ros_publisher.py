from contracts import contract
from reprep import scale, posneg, Report
from bootstrapping_olympics import Publisher
from ros import sensor_msgs
import rospy
from sensor_msgs.msg import Image #@UnresolvedImport
import numpy as np
from contextlib import contextmanager

class ROSPublisher(Publisher):
    ''' 
        This is the interface that the agents can use to publish
        debug information. 
    '''
    
    def __init__(self):      
        self.filters = {
            Publisher.FILTER_SCALE: scale,
            Publisher.FILTER_POSNEG: posneg,
        }
        
        self.ros_publishers = {} 
         
    def array(self, name, value):
        pass
    
    @contract(name='str', value='array')
    def array_as_image(self, name, value,
                               filter=Publisher.FILTER_POSNEG, filter_params={}):
        if not filter in self.filters:
            msg = 'Unknown filter %r; I know %s' % (filter, self.filters.keys())
            raise Exception(msg)
        rgb = self.filters[filter](value, **filter_params)
        ros_image = numpy_to_imgmsg(rgb, stamp=None) # XXX: stamp?
         
        #commands = np.kron(commands, np.ones((z, z)))
        if not name in self.ros_publishers:
            self.ros_publishers[name] = rospy.Publisher('~%s_image' % name, Image)

        self.ros_publishers[name].publish(ros_image)


    @contract(name='str', text='str')
    def text(self, name, text):
        ''' 
            Publishes a text object.

            Example: ::
            
                p.publish_text('status', 'I am ok')
        '''
        
    @contextmanager
    def plot(self, name, **args):
        r = Report()
        a = r.data_pylab('plot', **args)
        # XXX: better way?
        yield a.__enter__()
        a.__exit__(None, None, None)
        print r.children
    

def numpy_to_imgmsg(image, stamp=None):
    import sensor_msgs #@UnresolvedImport
    rosimage = sensor_msgs.msg.Image()
    rosimage.height = image.shape[0]
    rosimage.width = image.shape[1]
    if image.dtype == np.uint8:
        rosimage.encoding = '8UC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width
        rosimage.data = image.ravel().tolist()
    else:
        rosimage.encoding = '32FC%d' % image.shape[2]
        rosimage.step = image.shape[2] * rosimage.width * 4
        rosimage.data = np.array(image.flat, dtype=np.float32).tostring()
    if stamp is not None:
        rosimage.header.stamp = stamp
    return rosimage

