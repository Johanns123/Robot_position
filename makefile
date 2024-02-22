CC := g++
CFLAGS := -g -fdiagnostics-color=always -Wall
INCLUDE := -Iinclude
LIBS := -lm

SRC_DIR := src
BUILD_DIR := build

SRCS := $(wildcard $(SRC_DIR)/*.c)
EXEC := $(BUILD_DIR)/main.exe

.PHONY: all run clean

all: $(BUILD_DIR) $(EXEC)

$(EXEC): $(SRCS)
	$(CC) $(CFLAGS) $(INCLUDE) $^ -o $@ $(LIBS)

$(BUILD_DIR):
	mkdir -p $@

run: all
	./$(EXEC)

clean:
	rm -rf $(BUILD_DIR)
