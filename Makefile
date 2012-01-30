package=bootstrapping_olympics

all: develop

develop:
	python setup.py develop

docs: 
	make -C docs

nose=nosetests --with-id
nose_parallel=--processes=16 --process-timeout=30 --process-restartworker
nose_coverage=--with-coverage --cover-html --cover-html-dir coverage_information --cover-package=$(package)

test:
	@echo * Use NOSE_PARAMS to pass extra arguments.
	@echo * Use "make test-parallel" to enable parallel testing
	@echo * Use "make test-coverage" to do coverage testing
	@echo
	$(nose) $(package) $(NOSE_PARAMS)

test-parallel:
	@echo * Use NOSE_PARAMS to pass extra arguments.
	$(nose) $(package) $(nose_parallel) $(NOSE_PARAMS)

test-coverage:
	$(nose) $(package) $(nose_coverage) $(NOSE_PARAMS)


print-config:
	boot_olympics_print_config --outdir docs/source/my_static/config/