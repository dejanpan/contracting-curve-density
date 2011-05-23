SRC1 = ccd

all:
	pdflatex $(SRC1).tex
	bibtex $(SRC1)
	pdflatex $(SRC1).tex
	pdflatex $(SRC1).tex

clean:
	rm -f *.aux *.bbl *.blg *.log *.lof *.log *.lot *.out *.toc *~ *.dvi *.ps *.pdf
