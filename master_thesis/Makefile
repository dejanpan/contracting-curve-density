BASENAME = main
SUPPORTS = 

OTHER_FILES =

BIBFILE = #$(BASENAME).bib
BBLFILE = #$(BASENAME).bbl

LATEX = latex
LATEXFLAGS = -interaction=nonstopmode
BIBTEX = bibtex
PDFLATEX = pdflatex
DVIPS = dvips
DVIPSFLAGS = -t letter

ACROREAD = acroread
XDVI = xdvi
GGV = ggv

###############################################################################
# File lists

DISTFILES = *.aux *.log *~ *.blg *.bbl *.dvi
CLEANFILES = $(DISTFILES) *.ps *.pdf *.zip *.tar.gz *.tmp

TEMPLATE_FILES = Makefile $(BASENAME).tex

# Website

WEBSITE_FILES = $(BASENAME).pdf master_thesis.tar.gz \
	$(OTHER_FILES) $(TEMPLATE_FILES)

WEBSITE_PATH = 

###############################################################################
# Build rules

all: $(BASENAME).pdf

preview: $(BASENAME).dvi
	$(XDVI) $(BASENAME).dvi &

$(BASENAME).aux: $(BASENAME).tmp $(SUPPORTS) $(BIBFILE)
	$(LATEX) $(LATEXFLAGS) $(BASENAME).tex

$(BASENAME).bbl: $(BASENAME).tex $(BIBFILE) $(BASENAME).aux
	$(BIBTEX) $(BASENAME).aux
view: $(BASENAME).pdf
	acroread $< &

$(BASENAME).pdf:$(BASENAME).tex
	$(PDFLATEX) $(LATEXFLAGS) $<
	$(PDFLATEX) $(LATEXFLAGS) $<

###############################################################################
# Archive rules

archive: $(BASENAME).tar.gz

$(BASENAME).tar.gz: $(TEMPLATE_FILES)
	-mkdir master_thesis
	cp $(TEMPLATE_FILES) master_thesis
	tar zcvf master_thesis.tar.gz master_thesis
	-rm -rf master_thesis

###############################################################################
# Website Rules

website: $(WEBSITE_FILES)
	cp -avf $(WEBSITE_FILES) $(WEBSITE_PATH)

###############################################################################
# Clean-up rules

clean:
	-rm -f $(CLEANFILES)

distclean:
	-rm -f $(DISTFILES)


###############################################################################
# No longer used

# For RCS check-in and -out (Now under CVS control in a larger module)
RCSFILES = $(BASENAME).tex $(SUPPORTS) Makefile $(OTHER_FILES) \
	$(BASENAME).pdf $(BASENAME).ps master_thesis.tar.gz

ci: $(RCSFILES)
	$(shell for i in $(RCSFILES) ; do ci -u $$i ; done)

co: $(RCSFILES)
	$(shell for i in $(RCSFILES) ; do co -l $$i ; done)

# For archiving snapshots of the code (Moved to RCS, then CVS)
SNAPFILES = Makefile *.tex $(BASENAME).ps $(BASENAME).pdf $(BASENAME).dvi

snap: 
	-mkdir -p snapshots
	-mkdir `date +%Y%m%d`
	cp -R $(SNAPFILES) `date +%Y%m%d`
	tar zcvf snapshots/$(BASENAME).`date +%Y%m%d`.tar.gz `date +%Y%m%d`
	rm -rf `date +%Y%m%d`

# Used to make a distributable zip file
ZIPFILES = $(SUPPORTS) $(BASENAME).tex $(BIBFILE) $(BASENAME).ps \
	$(BASENAME).pdf $(OTHER_FILES)

zip: $(ZIPFILES)
	-rm -f $(BASENAME).zip
	zip $(BASENAME).zip $(ZIPFILES)

###############################################################################
.PHONY: check-syntax
check-syntax:
	$(LATEX) -file-line-error -no-pdf -interaction=nonstopmode '\newcommand{\pst}{false}\input{$<}' $(CHK_SOURCES) 
