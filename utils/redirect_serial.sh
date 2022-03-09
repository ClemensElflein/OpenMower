#!/bin/sh
trap 'kill %1; kill %2' SIGINT
/usr/bin/socat -b128 -d -d TCP-LISTEN:2101,fork,reuseaddr FILE:/dev/ttyAMA1,b115200,raw &
/usr/bin/socat -b128 -d -d TCP-LISTEN:2102,fork,reuseaddr FILE:/dev/ttyAMA2,b115200,raw &
/usr/bin/socat -b128 -d -d TCP-LISTEN:2103,fork,reuseaddr FILE:/dev/ttyAMA3,b115200,raw
