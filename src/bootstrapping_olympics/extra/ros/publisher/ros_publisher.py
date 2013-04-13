from reprep.constants import MIME_PNG
try:
    from .. import (rospy, ROSImage, Float32MultiArray, MultiArrayLayout,
    MultiArrayDimension)
except:  # allow to run nose even if ros is not installed
    pass
from bootstrapping_olympics import Publisher
from contextlib import contextmanager
from contracts import contract
from reprep import scale, posneg, Report
import numpy as np


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

    def get_publisher(self, name, data_class):
        """ Returns a publisher """        
        if not name in self.ros_publishers:
            topic = '~%s' % name
            rospy.loginfo('Creating new topic %r.' % topic)
            self.ros_publishers[name] = rospy.Publisher(topic, data_class)
        return self.ros_publishers[name]
        
    @contract(name='str|seq[>0](str)', value='array')
    def array(self, name, value):
        name = normalize(name)

        # TODO: generic dimensions
        v = np.array(value.flat, dtype='float32')
        msg = Float32MultiArray()
        msg.data = v.tolist()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension(label='no-label', size=v.size,
                                              stride=0)]

        pub = self.get_publisher(name, Float32MultiArray)
        pub.publish(msg)

    @contract(name='str|seq[>0](str)', value='array')
    def array_as_image(self, name, value,
                       filter=Publisher.FILTER_POSNEG,  # @ReservedAssignment 
                       filter_params={}):
        name = normalize(name)

        if not filter in self.filters:
            msg = 'Unknown filter %r; I know %s' % (filter,
                                                    self.filters.keys())
            raise Exception(msg)
        
        rgb = self.filters[filter](value, **filter_params)
        
        self.bitmap(name, rgb)

    def bitmap(self, name, rgb):
        ros_image = numpy_to_imgmsg(rgb, stamp=None)  # XXX: stamp?
        pub = self.get_publisher(name, ROSImage)
        pub.publish(ros_image)
        
        
    @contract(name='str|seq[>0](str)', text='str')
    def text(self, name, text):  # XXX: TODO:
        name = normalize(name)
        rospy.loginfo('Function text() not implemented')

    @contextmanager
    def plot(self, name, **args):
        name = normalize(name)

        r = Report()
        a = r.plot('plot', mime=MIME_PNG, **args)
        # XXX: better way?
        yield a.__enter__()
        a.__exit__(None, None, None)
        
        data_node = r.children[0]
        rgb = data_node.get_rgb()
        self.bitmap(name, rgb)



def rgb_from_pil(im):
    return np.asarray(im).astype(np.uint8)


def normalize(name):
    if isinstance(name, tuple):
        return '/'.join(name)
    name = name.replace('-', '_')
    return name


def numpy_to_imgmsg(image, stamp=None):
    rosimage = ROSImage()
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

