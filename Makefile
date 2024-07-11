.PHONY: cpp python

project-name = st_mapping

install:
	@python3 -m pip install --verbose python/

uninstall:
	@python3 -m pip uninstall $(project-name)

cpp:
	@cmake -Bbuild cpp/$(project-name)/
	@cmake --build build -j$(nproc -all)

run_cpp: cpp
	./build/apps/$(project-name)_maincpp

python:
	@cmake -Bbuild_python python/
	@cmake --build build_python -j$(nproc -all)

clean:
	rm -rf build/
	rm -rf build_python/
	rm -rf python/build/
