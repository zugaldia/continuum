.PHONY: install-lib uninstall-lib

install-lib:
	cd lib && pip install --break-system-packages -e .

uninstall-lib:
	pip uninstall --break-system-packages -y continuum
