#!/bin/sh

# This script starts OpenOCD on the pi and allows external connections.
# This way you can use VS code to remotely debug the Pico code.

echo "10" > /sys/class/gpio/export
echo "out" > /sys/class/gpio/gpio10/direction
echo "1" > /sys/class/gpio/gpio10/value
openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "bindto 0.0.0.0"

