PLATFORMIO       ?= uv run pio
PLATFORMIO_ENV   ?= uno_ladder_track
PLATFORMIO_ENVS  ?= uno_ladder_track uno_servo_test uno_button_test uno_i2c_scan uno_switch_board esp32_poe2_i2c_scan
JOBS             ?= $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 2)
BUILD_TARGETS    := $(addprefix build-,$(PLATFORMIO_ENVS))
UPLOAD_PORT      := /dev/ttyUSB0

all: build-all

.PHONY: build
build:
	$(PLATFORMIO) run -e $(PLATFORMIO_ENV)

.PHONY: build-all
build-all:
	$(MAKE) -j$(JOBS) $(BUILD_TARGETS)

build-%: FORCE
	$(PLATFORMIO) run -e $*

.PHONY: FORCE
FORCE:

.PHONY: compile_commands.json
compile_commands.json:
	$(PLATFORMIO) run -e $(PLATFORMIO_ENV) -t compiledb

.PHONY: upload
upload:
	$(PLATFORMIO) run -e $(PLATFORMIO_ENV) -t upload --upload-port $(UPLOAD_PORT)
