.PHONY: lint format baseline

lint:
	ruff check .
	mypy --ignore-missing-imports .

format:
	ruff format .

baseline:
	python3 baseline.py
