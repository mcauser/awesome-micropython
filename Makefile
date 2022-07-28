BASEDIR=$(CURDIR)
DOCDIR=$(BASEDIR)/docs

install:
	pip install mkdocs
	pip install mkdocs-minify-plugin
	pip install mkdocs-awesome-pages-plugin
	pip install mkdocs-material

link:
	ln -sf $(BASEDIR)/README.md $(DOCDIR)/index.md

serve:
	$(MAKE) link
	mkdocs serve

deploy:
	$(MAKE) link
	mkdocs gh-deploy --clean

