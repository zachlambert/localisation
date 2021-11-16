.PHONY: build_debug
build_debug:
	mkdir -p build/debug
	cmake -E chdir build/debug cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ../..
	cmake --build build/debug

.PHONY: run_debug
run_debug:
	build/debug/localisation

.PHONY: build_release
build_release:
	mkdir -p build/release
	cmake -E chdir build/release cmake -DCMAKE_BUILD_TYPE=Release ../..
	cmake --build build/release

.PHONY: run_release
run_release:
	build/release/localisation

.PHONY: build_profile
build_profile:
	mkdir -p build/profile
	cmake -E chdir build/profile cmake -DCMAKE_CXX_FLAGS=-pg ../..
	cmake --build build/profile

.PHONY: run_profile
run_profile:
	cd build/profile
	./localisation
