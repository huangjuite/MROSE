.PHONY: all cpp python uninstall

all: cpp

# Build standalone C++ library
cpp:
	cmake -S src -B build/cpp -DCMAKE_BUILD_TYPE=Release
	cmake --build build/cpp --parallel $(shell nproc)

# Install Python bindings
python:
	pip install .

# Editable Python install for development
editable:
	pip install scikit-build-core nanobind ninja cmake
	pip install --no-build-isolation -e .

uninstall:
	pip uninstall rpl-rio
