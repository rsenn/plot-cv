#!/bin/sh
ME="$0"
DIR=`dirname "$0"`

DIRS=$(
"$DIR"/find-cmake.sh "$@" |
  sed '/CMakeLists.txt/ s,/[^/]*$,,p' -n | sort -u
) 

set -- 

EXTS=".c .h .js .cmake"
EXCLUDES="*/build/* .git* */.git*"
set -f
for EXT in $EXTS; do
  if [ $# -gt 0 ]; then
    set -- "$@" -or -name "*$EXT"
  else
    set -- -name "*$EXT"
  fi
done

set --   "(" "$@" ")"

for EXCLUDE in $EXCLUDES; do
    set -- "$@" -and -not -wholename "$EXCLUDE"
done

set -- find $DIRS -type f "$@" 
exec "$@" 
