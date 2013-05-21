from abc import ABCMeta, abstractmethod

__all__ = ['LivePlugin']


class LivePlugin(object):
    __metaclass__ = ABCMeta

    """ 
        The interface for plugins that are run in parallel
        with the agent. Mostly used for visualization or 
        informal statistics.
    """
 
    @abstractmethod
    def init(self, init_data):
        """
            :param init_data: dictionary with fields
            
            Fields:
            - data_central
            - id_agent
            - id_robot 
        """

    def starting_stream(self, stream):
        """ A new stream is being read """
        pass

    @abstractmethod
    def update(self, update_data):
        """ 
            Called during interaction.
            
            :param update_data: dictionary with following fields:
            
            Fields:
            - agent
            - robot
            - obs
            - progress
            - state
            - stream
            
            Optional:
            - predict 
        """
        pass

    def finish(self):
        pass
