.PHONY: build
build:
	mkdir -p build
	cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
	cmake --build build

.PHONY: release
release:
	mkdir -p build_release
	cmake -E chdir build_release cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 ..
	cmake --build build_release

.PHONY: run
run:
	build/localisation
