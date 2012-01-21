
all:
	python setup.py install

develop:
	python setup.py develop

test:
	nosetests --with-id

test-parallel:
	nosetests --with-id --parallel=10

test-coverage:
	options="--with-coverage --cover-html --cover-html-dir coverage_information --cover-package=bootstrapping_olympics"
	nosetests --with-id  $(options) 

docs: 
	make -C docs

print-config:
	boot_olympics_print_config --outdir docs/source/my_static/config/