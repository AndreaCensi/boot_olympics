# v. 1.0

all: compile-website 
	
compile-website: website epydoc
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -E -n -a -b html source website


compile:
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -n -a -b html source website
	
website: distclean
	# Check out the website
	git clone git@github.com:AndreaCensi/boot_olympics.git  $@
	git --git-dir $@/.git --work-tree=$@/ checkout origin/gh-pages -b gh-pages
	git --git-dir $@/.git --work-tree=$@/ branch -D master

upload:
	git --git-dir $@/.git --work-tree=$@/ commit -m "updated website" -a 
	git --git-dir $@/.git --work-tree=$@/ push

distclean:
	rm -rf website/
	
epydoc:
	epydoc --config epydoc.cfg --introspect-only -v --exclude-introspect=bootstrapping_olympics.unittests --debug

