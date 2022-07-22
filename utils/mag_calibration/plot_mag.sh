#!/bin/sh
rostopic echo -b $1 -p /imu/mag | sed --expression 's/,/ /g' > /tmp/mag.csv
gnuplot -p plot_mag.gnuplot
