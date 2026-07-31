# The Pico SPI tap

RP2040 firmware that taps the bridge's SPI bus and resets the bridge. It replaces the Saleae Logic8 tap that `../docs/design/spi-tap.md` describes.

Design, wiring, accuracy budget and plan: `../docs/plans/pico-tap.md`.

## Upstream

Vendored from [`jjsch-dev/spi_sniffer_pio`](https://github.com/jjsch-dev/spi_sniffer_pio) by Juan Schiavoni, MIT licence. `LICENSE` is upstream's and stays with the code.

Changes against upstream:

- `TARGET_BUS_PIRATE_5` is 0, which selects the plain-Pico pin block.
- `SPI_TAP_ENABLE_PIN_CONFIG` is 0, so GP6 needs no wire. The gate has nothing to gate on: the bridge is powered whenever the tap is.

## Build

The laptop compiles and the Pi does not, which is the same split the receiver uses.

```
make pico1
```

Output lands in `bin/spi_sniffer_pio_rp2040.uf2`. The build needs Pico SDK 2.0.0 or later and reads `PICO_SDK_PATH` and `PICO_TOOLCHAIN_PATH` from the environment; `.zshrc` exports both.

Run `make clean` after any `cmake` configure failure. A half-configured `build/` makes the next run pick the host assembler for `boot_stage2`, which then fails on ARM Thumb instructions and reports a syntax error rather than the real cause.

## Flash

Hold BOOTSEL, plug the Pico in, and copy the `.uf2` onto the mass-storage volume that appears.
