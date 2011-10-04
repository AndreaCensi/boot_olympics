import sys
import traceback

def wrap_script_entry_point(function, logger):
    try:
        function(sys.argv[1:])
        sys.exit(0)
    except Exception as e:
        logger.error(str(e))
        logger.error(traceback.format_exc())
        sys.exit(-2) 
