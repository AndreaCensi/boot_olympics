from reprep import Report

Publisher = Report

# from contracts import contract 
# from abc import abstractmethod
# from contextlib import contextmanager
# from contracts import ContractsMeta
# 
# 
# __all__ = ['Publisher']
# 
# 
# class Publisher(object):
#     ''' 
#         This is the interface that the agents can use to publish
#         debug information. 
#     '''
# 
#     __metaclass__ = ContractsMeta
# 
#     @abstractmethod
#     @contract(name='str', value='array')
#     def array(self, name, value, caption=None):
#         ''' Publishes an array; the publisher decides how to visualize it. '''
# 
#     FILTER_POSNEG = 'posneg'
#     FILTER_SCALE = 'scale'
# 
#     @abstractmethod
#     @contract(name='str', value='array')
#     def array_as_image(self, name, value,
#                        filter='posneg', filter_params={},  # @ReservedAssignment
#                        caption=None):
#         ''' 
#             Publishes an array as a false-color image. 
#             ``filter`` is the name of a filter.
#             ``filter_params`` are parameters to pass to the filter.
#             
#             Usage example: ::
#             
#                 publisher.array_as_image('covariance', P, 'posneg')
#         '''
# 
#     @abstractmethod
#     @contract(name='str', text='str')
#     def text(self, name, text):
#         ''' 
#             Publishes a text object.
# 
#             Usage example: ::
#             
#                 publisher.text('status', 'I am ok')
#         '''
# 
#     @abstractmethod
#     def plot(self, name, caption=None, **args):
#         ''' 
#             Usage example: ::
#         
#                 with publisher.plot('my plot') as pylab:
#                     pylab.plot(Ey, label='E(y)')
#                     pylab.plot(y_max, label='y_max')
#                     pylab.plot(y_min, label='y_min')
#                     pylab.legend()
#         '''
# 
#     # TODO: make this abstract   
#     def section(self, section_name, caption=None, cols=None):  # @UnusedVariable
#         return Section(self, section_name)
# 
#     @contract(nid='None|valid_id', caption='None|str', robust='bool')
#     def subsection(self, nid=None, caption=None, robust=False):
#         """ use as context manager """
# 
# 
# 
# class Section():
#     def __init__(self, other, prefix):
#         self.other = other
#         self.prefix = prefix
# 
#     def concat(self, name):
#         base = [self.prefix]
#         if isinstance(name, str):
#             base.append(name)
#         else:
#             base.extend(name)
#         return "-".join(base)
# 
#     def array(self, name, *args):
#         return self.other.array(self.concat(name), *args)
# 
#     def array_as_image(self, name, *args, **kwargs):
#         return self.other.array_as_image(self.concat(name), *args, **kwargs)
# 
#     def text(self, name, text):
#         return self.other.text(self.concat(name), text)
# 
#     @contextmanager
#     def plot(self, name, **args):
#         with self.other.plot(self.concat(name), **args) as pylab:
#             yield pylab
# 
#     def section(self, section_name, caption=None):
#         return Section(self, section_name, caption=caption)

