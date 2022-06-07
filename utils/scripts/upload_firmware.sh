#!/bin/sh

# This script uploads new firmware to the Pico.
# You need to provide the .elf file, not the .uf2

echo "10" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio10/direction
echo "1" > /sys/class/gpio/gpio10/value
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program $1 verify reset exit"

