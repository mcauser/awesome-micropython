BASEDIR=$(CURDIR)
DOCDIR=$(BASEDIR)/docs

install:
	pip install mkdocs
	pip install mkdocs-minify-plugin
	pip install mkdocs-awesome-pages-plugin

link:
	ln -sf $(BASEDIR)/README.md $(DOCDIR)/index.md
	ln -sf $(BASEDIR)/logo.svg $(DOCDIR)/img/logo.svg

serve:
	$(MAKE) link
	mkdocs serve

deploy:
	$(MAKE) link
	mkdocs gh-deploy --clean