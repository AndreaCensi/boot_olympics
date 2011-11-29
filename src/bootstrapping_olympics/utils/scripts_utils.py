__version__ = '1.0'

import sys
import traceback

class UserError(Exception):
    pass

def wrap_script_entry_point(function, logger):
    try:
        function(sys.argv[1:])
        sys.exit(0)
    except UserError as e:
        logger.error(str(e))
        sys.exit(1) 
    except Exception as e:
        logger.error(traceback.format_exc())
        sys.exit(-55) 
