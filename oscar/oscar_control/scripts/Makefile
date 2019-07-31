christine_hwi_ext.so: christine_hwi_ext.pyx setup_christine_hwi_ext.py
	python setup_christine_hwi_ext.py build_ext --inplace


pyrobcape.so: robcape.pyx setup_robcape_ext.py
	@echo "## Building sbpl python extension ##"
	python setup_robcape_ext.py build_ext --inplace
	@echo "## all done##\n"

clean:
	rm -rf *~ *.pyc robcape.cpp pyrobcape.so christine_hwi_ext.so christine_hwi_ext.cpp build
