#!/bin/sh
THISDIR=`dirname "$0"`

main() {
  if [ $# -le 0 ]; then
    set -- "$THISDIR/grbl_v1.1f.20170801.hex"
  fi
 (set -x
 avrdude -c arduino -b 57600 -P /dev/ttyUSB0 -p atmega328p -vv -U "$@")
}

main "$@"
