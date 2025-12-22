.PHONY: install-lib uninstall-lib lint format

install-lib:
	cd lib && pip install --break-system-packages -e .

uninstall-lib:
	pip uninstall --break-system-packages -y continuum

lint:
	cd lib && $(MAKE) lint
	cd workspace && $(MAKE) lint

format:
	cd lib && $(MAKE) format
	cd workspace && $(MAKE) format
