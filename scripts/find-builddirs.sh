#!/bin/sh
MYDIR=`dirname "$0"`
PDIR=`dirname "$MYDIR"`
#echo "PDIR='$PDIR'" 1>&2
ls -d $(find ${@:-$PDIR} -type d -name build | sed -u 's|$|/*-*-*/|') 2>/dev/null |
  sed -u 's|^\./||'
