.PHONY: all cpp

all: cpp

BUILD_DIR ?= build/cpp
BUILD_PYTHON_BINDINGS ?= OFF

# Build standalone C++ library
cpp:
	cmake -S src -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=$(BUILD_PYTHON_BINDINGS)
	cmake --build $(BUILD_DIR) --parallel $(shell nproc)
