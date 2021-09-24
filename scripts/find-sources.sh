#!/bin/sh
ME=$0
MYDIR=`dirname "$0"`
THISDIR=${MYDIR%/scripts}

find_sources() {
   if [ $# -le 0 ]; then
     SAVE_PWD=`pwd`
     cd "$THISDIR"
     for SUBDIR in qjs-* quickjs quickjs/qjs-*; do
       test -d "$SUBDIR" &&
       set -- "$@" "$THISDIR/$SUBDIR"
     done
     cd "$SAVE_PWD"
   fi

   find "$@" -maxdepth 1 -name "*.[ch]" -or -name "*.[ch]pp" |
     sed -u 's|^\./||'
}

find_sources "$@"
