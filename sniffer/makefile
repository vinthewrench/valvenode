#
# makefile for ValveNode RS-485 sniffer
#

TARGET := valvenode_sniff

SRC_DIR := src
BUILD_DIR := build

SRC := $(SRC_DIR)/valvenode_sniff.cpp
OBJ := $(BUILD_DIR)/valvenode_sniff.o

CXX := clang++
CXXFLAGS := -std=c++17 -Wall -Wextra -Wpedantic -O2
LDFLAGS :=

.PHONY: all clean rebuild run

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

$(OBJ): $(SRC) | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

run: $(TARGET)
	./$(TARGET)

clean:
	rm -rf $(BUILD_DIR) $(TARGET)

rebuild: clean all
