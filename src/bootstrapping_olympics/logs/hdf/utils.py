import zlib
from bootstrapping_olympics.utils.c_yaml import yaml_load


def load_extra(extra_table, index):
    extra_string = str(extra_table[index])
    extra_string = zlib.decompress(extra_string)
    extra = yaml_load(extra_string)
    return extra
