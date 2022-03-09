#!/bin/sh
/usr/bin/socat -b128 -d -d TCP-LISTEN:2102,fork,reuseaddr FILE:$1,b115200,raw
