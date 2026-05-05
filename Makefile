.PHONY: all cpp

all: cpp

# Build standalone C++ library
cpp:
	cmake -S src -B build/cpp -DCMAKE_BUILD_TYPE=Release
	cmake --build build/cpp --parallel $(shell nproc)
