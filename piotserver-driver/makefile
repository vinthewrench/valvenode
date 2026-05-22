PLUGIN_NAME := VALVEMASTER
PLUGIN_DIR  := drivers/$(PLUGIN_NAME)

UNAME_S := $(shell uname -s)

ifeq ($(UNAME_S),Darwin)
	PLUGIN_EXT := dylib
else
	PLUGIN_EXT := so
endif

.PHONY: all clean run plugin harness dirs

all: dirs plugin harness

dirs:
	mkdir -p build
	mkdir -p plugins

plugin:
	$(MAKE) -C $(PLUGIN_DIR)

harness: dirs
	$(MAKE) -C test

run: all
	./build/plugin_harness ./plugins/$(PLUGIN_NAME).$(PLUGIN_EXT)

clean:
	$(MAKE) -C $(PLUGIN_DIR) clean
	$(MAKE) -C test clean
	rm -rf build
	rm -f plugins/$(PLUGIN_NAME).so
	rm -f plugins/$(PLUGIN_NAME).dylib
