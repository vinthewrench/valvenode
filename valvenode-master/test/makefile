# Valve Master I2C test tool
# Plain Makefile version of the existing clang/clang++ CMake style.

APP       := valve
SRC_DIR   := src
BUILD_DIR := build

CC        := clang
CXX       := clang++

CPPFLAGS  := -I$(SRC_DIR)
CXXFLAGS  := -std=c++17 -Wall -Wextra -O2 -g
LDFLAGS   :=
LDLIBS    :=

SRCS := \
	$(SRC_DIR)/main.cpp \
	$(SRC_DIR)/I2C.cpp \
	$(SRC_DIR)/TimeStamp.cpp \
	$(SRC_DIR)/LogMgr.cpp		\
	$(SRC_DIR)/Valve_master.cpp


OBJS := $(SRCS:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)
DEPS := $(OBJS:.o=.d)

.PHONY: all clean run

all: $(APP)

$(APP): $(OBJS)
	$(CXX) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -MMD -MP -c $< -o $@

run: $(APP)
	./$(APP)

clean:
	rm -rf $(BUILD_DIR) $(APP)

-include $(DEPS)
