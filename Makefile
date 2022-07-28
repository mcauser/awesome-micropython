BASEDIR=$(CURDIR)
DOCDIR=$(BASEDIR)/docs

install:
	pip install mkdocs
	pip install mkdocs-minify-plugin
	pip install mkdocs-awesome-pages-plugin
	pip install mkdocs-material
	pip install mkdocs-include-markdown-plugin

serve:
	mkdocs serve

deploy:
	$(MAKE) link
	mkdocs gh-deploy --clean
