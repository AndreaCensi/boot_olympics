from conf_tools import ConfigMaster

__all__ = ['BatchConfigMaster']

class BatchConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self)
        self.add_class('sets', '*.sets.yaml', check_valid_set_config)
        self.sets = self.specs['sets']

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")


def check_valid_set_config(struct):
    pass


