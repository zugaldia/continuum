# Note: For library-specific commands, see lib/Makefile
# For workspace-specific commands, see workspace/Makefile
# For scripts-specific commands, see scripts/Makefile

.PHONY: install-lib install-models uninstall-lib lint format

install-lib:
	cd lib && pip install --break-system-packages -e .

# Install language models for TTS (kokoro/misaki)
# Note: For Spanish/French/Hindi/Italian/Portuguese support, also install: sudo apt install espeak-ng
install-spacy-models:
	python3 -m spacy download en_core_web_sm --break-system-packages

uninstall-lib:
	pip uninstall --break-system-packages -y continuum

lint:
	cd lib && $(MAKE) lint
	cd workspace && $(MAKE) lint
	cd scripts && $(MAKE) lint

format:
	cd lib && $(MAKE) format
	cd workspace && $(MAKE) format
	cd scripts && $(MAKE) format
