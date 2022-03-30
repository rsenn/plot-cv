#!/bin/sh

MYDIR="`dirname "$0"`"
PDIR="`dirname "$MYDIR"`"
#echo "PDIR='$PDIR'" 1>&2

DIRS=
OPTS=
NL="
"
while [ $# -gt 0 ]; do
  if [ -d "$1" ]; then
    DIRS="${DIRS:+$DIRS$NL}$1"
    shift
    continue
  fi
    break
done
while [ $# -gt 0 ]; do
  case "$1" in
    -name|-iname) OPTS="${OPTS:+$OPTS$NL}${1/name/wholename}$NL$2/CMakeCache.txt"
      shift ;;
    *) OPTS="${OPTS:+$OPTS$NL}$1" ;;
  esac
    shift
done


BUILDDIRS=$(ls -d $(find ${DIRS:-$PDIR} -type d -name build | sed -u 's|$|/*-*-*/|') 2>/dev/null | sed -u 's|^./||')

exec find $BUILDDIRS $OPTS -type f -name CMakeCache.txt -exec dirname {} \;
