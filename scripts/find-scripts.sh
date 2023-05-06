#!/bin/sh
ME=$0
MYDIR=`dirname "$0"`
THISDIR=${MYDIR%/scripts}

find_sources() {
   if [ $# -le 0 ]; then
     SAVE_PWD=`pwd`
     cd "$THISDIR/.."
     for SUBDIR in {.,qjs-*,quickjs,quickjs/qjs-*}{,/{lib,examples,tests}}; do
       test -d "$SUBDIR" &&
       set -- "$@" "$SUBDIR"
     done
     cd "$SAVE_PWD"
   fi

   find "$@" -maxdepth 1 -name "*.js" |
     sed -u 's|^\./||'
}

find_sources "$@"
