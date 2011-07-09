import os
import fnmatch
import yaml #@UnresolvedImport
from contracts import check
from . import logger
from contracts import contract
from pprint import pformat

def locate_files(directory, pattern):
    for root, dirs, files in os.walk(directory): #@UnusedVariable
        for f in files: 
            if fnmatch.fnmatch(f, pattern):
                yield os.path.join(root, f)


@contract(directory='str',
          pattern='str',
          required_fields='list(str)',
          optional_fields='list(str)')
def load_configuration_entries(directory, pattern='*.yaml',
                               required_fields=[],
                               optional_fields=[],
                               check_not_allowed=True):
    ''' 
        Loads all .dynamics.yaml files recursively in the directory. 
        It is assumed that each file contains a list of dictionaries.
        Each dictionary MUST have the field 'id' and it must be unique.
        Moreover, it MUST have other fields as specified in required_fields.
        Moreover, it CANNOT have other fields other than 'id', 
        required_fields, optional_fields. 
    '''
    
    def enumerate_entries():
        for filename in locate_files(directory, pattern):
            with open(filename) as f:
                parsed = yaml.load(f)
                if parsed is None:
                    raise Exception('Empty file %r.' % filename) 
                check('list(dict)', parsed)
                if not parsed:
                    raise Exception('Empty file %r.' % filename) 
                for entry in parsed: 
                    yield filename, entry 
    
    allowed_fields = set(['id'] + required_fields + optional_fields)
    required_fields = set(['id'] + required_fields)
    def check_entry(x):
        for field in required_fields:
            if not field in x:
                raise Exception('Entry does not have field %r.' % (field))
        if check_not_allowed:
            for found in x:
                if not found in allowed_fields:
                    msg = 'Field %r not allowed in entry.' % (found)
                    raise Exception(msg)        
        
    all_entries = {}
    for filename, x in enumerate_entries():
        try: 
            check_entry(x)
            name = x['id']
            if name in all_entries:
                raise Exception('Already know %r from %r' % 
                                (name, all_entries[name]['filename']))
            x['filename'] = filename
            all_entries[name] = x
        except Exception as e:
            msg = ('Error while loading entry from file %r.\n%s\nError: %s' % 
                   (filename, pformat(x), e))
            logger.error(msg) # XXX
            raise    
    return all_entries
    
