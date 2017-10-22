pyrobcape.so: robcape.pyx setup_robcape_ext.py
	@echo "## Building sbpl python extension ##"
	python setup_robcape_ext.py build_ext --inplace
	@echo "## all done##\n"

clean:
	rm -rf *~ *.pyc robcape.cpp pyrobcape.so build
