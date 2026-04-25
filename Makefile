# Makefile — voloop digital power control algorithm library
#
# Targets:
#   all          Build all example programs (default)
#   examples     Same as all
#   test         Build and run examples, check exit codes
#   clean        Remove build artifacts

CC      ?= gcc
CFLAGS  ?= -Wall -Wextra -Wpedantic -std=c99 -O2
IFLAGS   = -Iinclude

SRC_DIR     = src
EXAMPLE_DIR = example
BUILD_DIR   = build

# Library source files
LIB_SRCS = $(SRC_DIR)/voloop_pid.c \
           $(SRC_DIR)/voloop_compensator.c \
           $(SRC_DIR)/voloop_filter.c

# Example programs
EXAMPLES = $(BUILD_DIR)/basic_pid_voltage_loop \
           $(BUILD_DIR)/advanced_compensator_loop

.PHONY: all examples test clean

all: examples

examples: $(BUILD_DIR) $(EXAMPLES)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(BUILD_DIR)/basic_pid_voltage_loop: \
        $(EXAMPLE_DIR)/basic_pid_voltage_loop.c \
        $(SRC_DIR)/voloop_pid.c \
        include/voloop_pid.h include/voloop.h
	$(CC) $(CFLAGS) $(IFLAGS) \
	    $(SRC_DIR)/voloop_pid.c \
	    $< -o $@ -lm

$(BUILD_DIR)/advanced_compensator_loop: \
        $(EXAMPLE_DIR)/advanced_compensator_loop.c \
        $(SRC_DIR)/voloop_compensator.c \
        $(SRC_DIR)/voloop_filter.c \
        include/voloop_compensator.h include/voloop_filter.h include/voloop.h
	$(CC) $(CFLAGS) $(IFLAGS) \
	    $(SRC_DIR)/voloop_compensator.c \
	    $(SRC_DIR)/voloop_filter.c \
	    $< -o $@ -lm

test: examples
	@echo "--- Running basic_pid_voltage_loop ---"
	$(BUILD_DIR)/basic_pid_voltage_loop
	@echo "--- Running advanced_compensator_loop ---"
	$(BUILD_DIR)/advanced_compensator_loop
	@echo "--- All examples passed ---"

clean:
	rm -rf $(BUILD_DIR)
