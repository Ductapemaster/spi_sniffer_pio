# Cambiamos el nombre al target real de tu CMake
TARGET = spi_sniffer_pio

.PHONY: all clean pico1 pico2

all: pico1

# Target para la Pico 1 (RP2040)
pico1:
	mkdir -p bin
	cmake -B build -DPICO_BOARD=pico
	$(MAKE) -C build
	cp build/$(TARGET).uf2 bin/$(TARGET)_rp2040.uf2

# Target para la Pico 2 (RP2350)
pico2:
	mkdir -p bin
	cmake -B build -DPICO_BOARD=pico2
	$(MAKE) -C build
	cp build/$(TARGET).uf2 bin/$(TARGET)_rp2350.uf2

clean:
	rm -rf build/
	rm -rf bin/
