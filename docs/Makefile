# v. 1.0
webdir=website
webgit=git --git-dir $(webdir)/.git --work-tree=$(webdir)/ 
	
all: upload
	
compile-website: website epydoc
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -E -n -a -b html source website


compile:
	PYTHONPATH=`pwd`:$(PYTHONPATH) sphinx-build -n -a -b html source website
	
website: distclean
	# Check out the website
	git clone git@github.com:AndreaCensi/boot_olympics.git  $(webdir)
	$(webgit) checkout origin/gh-pages -b gh-pages
	$(webgit) branch -D master

upload: compile-website
	./gitadd.zsh
	$(webgit) push

distclean:
	rm -rf website/
	
epydoc:
	epydoc --config epydoc.cfg --introspect-only -v --exclude-introspect=bootstrapping_olympics.unittests --debug

