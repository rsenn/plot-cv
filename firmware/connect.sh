#!/bin/sh
exec gtkterm --port /dev/ttyUSB0 --speed 115200 --echo "$@"