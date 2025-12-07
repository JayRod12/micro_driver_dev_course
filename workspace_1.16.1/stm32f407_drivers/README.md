# STM32F407 Drivers

## Switching Exercises

Each exercise has its own `main_*.c` file in `Src/`. The active one is symlinked to `main.c`.

```bash
# List available exercises
ls Src/main_*.c

# Switch to a different exercise
ln -sf main_gpio_led.c Src/main.c

# Rebuild
cd Debug && make clean all
```

## Building

```bash
cd Debug
make -j$(nproc)
```

## Flashing

```bash
# Generate .bin and flash
arm-none-eabi-objcopy -O binary Debug/stm32f407_drivers.elf Debug/stm32f407_drivers.bin
st-flash write Debug/stm32f407_drivers.bin 0x08000000
```

Or via GDB:
```bash
# Option A: OpenOCD (run in separate terminal first: openocd -f interface/stlink.cfg -f target/stm32f4x.cfg)
arm-none-eabi-gdb Debug/stm32f407_drivers.elf -ex "target remote :3333" -ex "load"

# Option B: st-util (run in separate terminal first: st-util)
arm-none-eabi-gdb Debug/stm32f407_drivers.elf -ex "target remote :4242" -ex "load"
```

## Debugging

Terminal 1 - Start OpenOCD:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
```

Terminal 2 - GDB:
```bash
arm-none-eabi-gdb Debug/stm32f407_drivers.elf
(gdb) target remote :3333
(gdb) load
(gdb) break main
(gdb) continue
```

Useful GDB commands:
- `load` - flash program
- `monitor reset run` - reset and run
- `c` - continue
- `n` - step over
- `s` - step into
- `Ctrl+C` - halt

## SWO Printf

Printf output goes via SWO (configured in `syscalls.c`). View with:
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "init; itm port 0 on; tpiu config internal - uart off 16000000"
```
