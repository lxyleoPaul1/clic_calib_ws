MAIN = main
LATEX = pdflatex
BIBTEX = bibtex

.PHONY: all clean pdf

all: pdf

pdf: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex sec_*.tex refs.bib
	$(LATEX) -interaction=nonstopmode $(MAIN).tex
	-$(BIBTEX) $(MAIN)
	$(LATEX) -interaction=nonstopmode $(MAIN).tex
	$(LATEX) -interaction=nonstopmode $(MAIN).tex

clean:
	rm -f $(MAIN).pdf $(MAIN).aux $(MAIN).bbl $(MAIN).blg $(MAIN).log $(MAIN).out
