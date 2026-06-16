.PHONY: install run debug clean lint lint-strict test

PYTHON = python3
MAIN   = main.py
MAP    ?= map.txt

install:
	pip install flake8 mypy pytest --break-system-packages

run:
	cd fly_in && $(PYTHON) $(MAIN) $(MAP)

debug:
	cd fly_in && $(PYTHON) -m pdb $(MAIN) $(MAP)

clean:
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type d -name .mypy_cache -exec rm -rf {} + 2>/dev/null || true
	find . -name "*.pyc" -delete 2>/dev/null || true

lint:
	cd fly_in && flake8 . --max-line-length=100 && \
	mypy . --warn-return-any --warn-unused-ignores \
	        --ignore-missing-imports --disallow-untyped-defs \
	        --check-untyped-defs

lint-strict:
	cd fly_in && flake8 . --max-line-length=100 && mypy . --strict

test:
	cd fly_in && $(PYTHON) -m pytest tests/ -v