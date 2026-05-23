# Root makefile for the ValveNode repository.
#
# Builds and cleans:
#
#   - valvenode-master firmware
#   - valvenode-master simulator / test tooling
#   - valvenode-slave firmware
#   - valvenode-slave test tooling
#   - piotserver-driver plugin and harness
#
# Intent:
#
#   make          build everything useful
#   make clean    remove generated build/checkin junk
#   make status   show git status after cleaning
#
# Notes:
#
#   - AVR firmware subdirectories should use their own AVR toolchain makefiles.
#   - Pi/Linux C++ code should use clang/clang++ inside its own makefiles.
#   - This root makefile does not try to second-guess subproject internals.

SHELL := /bin/bash

SUBDIRS := \
	valvenode-master/firmware \
	valvenode-master/sim \
	valvenode-master/test \
	valvenode-slave/firmware \
	valvenode-slave/test \
	piotserver-driver

.PHONY: all build clean distclean status check dirs \
	master master-clean \
	master-firmware master-firmware-clean \
	master-sim master-sim-clean \
	master-test master-test-clean \
	slave slave-clean \
	slave-firmware slave-firmware-clean \
	slave-test slave-test-clean \
	piotserver-driver piotserver-driver-clean

all: build

build: dirs
	@set -e; \
	for dir in $(SUBDIRS); do \
		echo "==> Building $$dir"; \
		$(MAKE) -C "$$dir"; \
	done

dirs:
	@mkdir -p build

clean:
	@set -e; \
	for dir in $(SUBDIRS); do \
		echo "==> Cleaning $$dir"; \
		$(MAKE) -C "$$dir" clean || true; \
	done
	@echo "==> Removing root generated build artifacts"
	@rm -rf build
	@find . -name '*.o' -type f -delete
	@find . -name '*.d' -type f -delete
	@find . -name '*.gcda' -type f -delete
	@find . -name '*.gcno' -type f -delete
	@find . -name '*.gcov' -type f -delete
	@find . -name '.DS_Store' -type f -delete
	@find . -name '*~' -type f -delete

distclean: clean
	@echo "==> Removing generated binaries and plugin artifacts"
	@rm -rf piotserver-driver/plugins
	@rm -f piotserver-driver/build/plugin_harness
	@rm -f valvenode-master/test/valve
	@rm -f valvenode-slave/test/vnode
	@rm -f valvenode-slave/firmware/valvenode.elf
	@rm -f valvenode-slave/firmware/valvenode.hex
	@rm -f valvenode-slave/firmware/valvenode.map
	@rm -f valvenode-master/firmware/valvenode_master.elf
	@rm -f valvenode-master/firmware/valvenode_master.hex
	@rm -f valvenode-master/firmware/valvenode_master.map

status:
	@git status --short

check: clean status

master: master-firmware master-sim master-test

master-clean: master-firmware-clean master-sim-clean master-test-clean

master-firmware:
	$(MAKE) -C valvenode-master/firmware

master-firmware-clean:
	$(MAKE) -C valvenode-master/firmware clean || true

master-sim:
	$(MAKE) -C valvenode-master/sim

master-sim-clean:
	$(MAKE) -C valvenode-master/sim clean || true

master-test:
	$(MAKE) -C valvenode-master/test

master-test-clean:
	$(MAKE) -C valvenode-master/test clean || true

slave: slave-firmware slave-test

slave-clean: slave-firmware-clean slave-test-clean

slave-firmware:
	$(MAKE) -C valvenode-slave/firmware

slave-firmware-clean:
	$(MAKE) -C valvenode-slave/firmware clean || true

slave-test:
	$(MAKE) -C valvenode-slave/test

slave-test-clean:
	$(MAKE) -C valvenode-slave/test clean || true

piotserver-driver:
	$(MAKE) -C piotserver-driver

piotserver-driver-clean:
	$(MAKE) -C piotserver-driver clean || true
