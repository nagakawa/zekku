CPP=c++ \
	-Iinclude/ -I/usr/include/ -I3rdparty/kozet_fixed_point/include/ \
	-DUSE_GLM --std=c++14
CFLAGS=-Wall -Werror -pedantic -Og -g
CFLAGS_PROFILED=-Wall -Werror -pedantic -Og -g -lprofiler
CFLAGS_RELEASE=-Wall -Werror -pedantic -O3 -march=native

all: build/test

build/test: test/main.cpp \
		include/zekku/Pool.h \
		include/zekku/geometry.h \
		include/zekku/QuadTree.h \
		include/zekku/BoxQuadTree.h \
		include/zekku/bitwise.h \
		include/zekku/BloomFilter.h \
		include/zekku/base.h \
		include/zekku/timath.h \
		include/zekku/kfp_interop/timath.h \
		3rdparty/kozet_fixed_point/include/kozet_fixed_point/kfp.h \
		3rdparty/kozet_fixed_point/include/kozet_fixed_point/kfp_extra.h
	@mkdir -p build
	@echo -e '\e[33mCompiling test program...\e[0m'
	@$(CPP) --std=c++14 test/main.cpp -o build/test $(CFLAGS_RELEASE)
	@echo -e '\e[32mDone!\e[0m'

clean:
	rm build/test
