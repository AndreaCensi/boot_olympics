import os
import fnmatch
import yaml
from contracts import check 
from .utils import instantiate_spec

def locate_files(directory, pattern):
    for root, dirs, files in os.walk(directory): #@UnusedVariable
        for f in files: 
            if fnmatch.fnmatch(f, pattern):
                yield os.path.join(root, f)


def load_config_dynamics(directory, pattern='*.dynamics.yaml'):
    ''' Loads all .dynamics.yaml files recursively in the directory. '''
    
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
    
    def load_entry(x):
        necessary = ['id', 'desc', 'code']
        for field in necessary:
            if not field in x:
                raise Exception('Entry does not have field %r.' % field)
            
        id = x['id']
        instance = instantiate_spec(x['code'])
        return x['id'], instance        
        
    all_entries = {}
    for filename, x in enumerate_entries():
        try: 
            name, dynamics = load_entry(x)
            if name in all_entries:
                raise Exception('Already know %r from %r' % 
                                (name, all_entries[name]['filename']))
            all_entries[name] = dynamics
        except Exception as e:
            msg = ('Error while loading entry from %r.\nEntry: %r\nError: %s' % 
                   (filename, x, e))
            print(msg) # XXX
            raise    
    return all_entries
    
    
def load_config_dynamics_demo():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option("-d", dest="directory",
                      help="base directory for configuration", metavar="FILE")
    (options, args) = parser.parse_args()

    if args: 
        raise Exception('Spurious arguments')
    if options.directory is None:
        raise Exception('Please pass -d')
    
    dynamics = load_config_dynamics(options.directory)
    
    for id, x in dynamics.items():
        print('%20s: %s' % (id, x)) 
    
if __name__ == '__main__':
    load_config_dynamics_demo()
