CC        := clang
CXX       := clang++

ROOT_DIR  := ..
SRC_DIR   := $(ROOT_DIR)/src
BUILD_DIR := $(ROOT_DIR)/build/test
TARGET    := $(ROOT_DIR)/build/plugin_harness

CPPFLAGS  := -isystem $(SRC_DIR)
CXXFLAGS  := -std=gnu++23 -Wall -Wextra -Wpedantic -O2 -g
LDFLAGS   :=
LDLIBS    :=

UNAME_S := $(shell uname -s)

ifneq ($(UNAME_S),Darwin)
	LDFLAGS += -rdynamic
	LDLIBS += -ldl
endif

OBJS := \
	$(BUILD_DIR)/plugin_harness.o \
	$(BUILD_DIR)/LogMgr.o \
	$(BUILD_DIR)/TimeStamp.o

.PHONY: all clean dirs

all: dirs $(TARGET)

dirs:
	mkdir -p $(BUILD_DIR)

$(TARGET): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD_DIR)/plugin_harness.o: plugin_harness.cpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/LogMgr.o: $(SRC_DIR)/LogMgr.cpp $(SRC_DIR)/LogMgr.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/TimeStamp.o: $(SRC_DIR)/TimeStamp.cpp $(SRC_DIR)/TimeStamp.hpp
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD_DIR)
	rm -f $(TARGET)
