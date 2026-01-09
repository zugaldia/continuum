# Note: For library-specific commands, see lib/Makefile
# For workspace-specific commands, see workspace/Makefile
# For scripts-specific commands, see scripts/Makefile

.PHONY: install-lib install-models uninstall-lib lint format docker-build docker-rebuild docker-clean docker-up docker-down docker-logs

install-lib:
	cd lib && pip install --break-system-packages -e .[mic,vad]

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

copy-test-audio:
	mkdir -p ${HOME}/.local/share/continuum/reachy
	cp assets/audio/jfk.wav ${HOME}/.local/share/continuum/reachy/test.wav

docker-build:
	docker compose build

docker-rebuild:
	docker compose build --no-cache

docker-up:
	docker compose up -d

docker-down:
	docker compose down

docker-logs:
	docker compose logs -f
