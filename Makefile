
all:
	python setup.py install

develop:
	python setup.py develop

test:
	nosetests

test-parallel:
	nosetests --parallel=10

docs: 
	make -C docs

print-config:
	boot_olympics_print_config --outdir docs/source/my_static/config/