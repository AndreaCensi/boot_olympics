from bootstrapping_olympics.configuration.batch_config import get_conftools_bootbatchsets
from comptests import comptests_for_all

library_sets = get_conftools_bootbatchsets()
for_all_sets = comptests_for_all(library_sets)
