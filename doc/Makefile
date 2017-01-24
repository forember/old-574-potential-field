NAME=potential_field
USE_LINECONTROL=yes
USE_WEBKIT=

all: catkin $(NAME).html $(NAME).pdf

catkin: ../src/$(NAME).cpp
	bash -c 'cd ../../.. && catkin_make'

../src/%.cpp: tools/.%.tmp.lit
	bash -c 'cd .. && lit --tangle doc/$<'

ifdef USE_LINECONTROL
tools/.%.tmp.lit: %.lit tools/linecontrol.py
	tools/linecontrol.py $< > $@
else
tools/.%.tmp.lit: %.lit
	cp $< $@
endif

ifdef USE_WEBKIT
%.pdf: %.html
	wkhtmltopdf -s Letter $< $@
else
%.pdf: %.html tools/myhtml2pdf.bash tools/applykatex.js tools/pdfLatexHeader.tex
	tools/myhtml2pdf.bash $< $@
endif

%.html: %.lit tools/add.css
	lit --weave $<

clean: cleancatkin

cleanall: cleancatkin cleanlit

cleancatkin:
	bash -c 'cd ../../.. && catkin_make clean'

cleanlit: $(NAME).lit
	rm -f ../src/$(NAME).cpp $(NAME).html $(NAME).pdf tools/.$(NAME).tmp.lit
