# Wrapper Makefile to automate CMake builds from the root directory
BUILD_DIR = build

.PHONY: all clean

all:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake ..
	@cmake --build $(BUILD_DIR)

clean:
	@echo "Cleaning project build directory..."
	@rm -rf $(BUILD_DIR)
