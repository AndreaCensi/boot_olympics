from boot_manager import get_conftools_bootbatchsets
from comptests import comptests_for_all, comptests_for_all_dynamic

library_sets = get_conftools_bootbatchsets()
for_all_sets = comptests_for_all(library_sets)
for_all_sets_dynamic = comptests_for_all_dynamic(library_sets)
