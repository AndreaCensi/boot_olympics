from conf_tools import ConfigMaster
from contracts import contract
from conf_tools.objspec import ObjectSpec

__all__ = ['BatchConfigMaster']

class BatchConfigMaster(ConfigMaster):
    def __init__(self):
        ConfigMaster.__init__(self)
        self.add_class('sets', '*.sets.yaml', check_valid_set_config)
        self.sets = self.specs['sets']

    def get_default_dir(self):
        from pkg_resources import resource_filename  # @UnresolvedImport
        return resource_filename("bootstrapping_olympics", "configs")

get_bootbatch_config = BatchConfigMaster.get_singleton

@contract(returns=ObjectSpec)
def get_conftools_bootbatchsets():
    return get_bootbatch_config().agents

def check_valid_set_config(struct):
    pass


