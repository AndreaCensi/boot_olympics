
all:
	python setup.py install

develop:
	python setup.py develop

test:
	nosetests

docs: epydoc

epydoc:
	epydoc --config epydoc.cfg --introspect-only  -v --exclude-introspect=bootstrapping_olympics.unittests --debug
