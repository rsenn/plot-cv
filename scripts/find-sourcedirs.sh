#!/bin/sh

MYDIR="`dirname "$0"`"
PDIR="`dirname "$MYDIR"`"
#echo "PDIR='$PDIR'" 1>&2

NL="
"
DEFAULT_OPTS=$(IFS="$NL"; set -f; set -- "(" \
  -not -wholename "*/build/*" \
  -and \
  -not -wholename "*/inst/*" \
  -and \
  -not -wholename "*/.git*" \
  ")"; echo "$*")

DIRS=
OPTS=

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
    -mindepth|-maxdepth) LEADING_OPTS="${LEADING_OPTS:+$LEADING_OPTS$NL}${1}$NL${2}"
      shift ;;
    -name|-iname) OPTS="${OPTS:+$OPTS$NL}${1/name/wholename}$NL$2/CMakeLists.txt"
      shift ;;
    *) OPTS="${OPTS:+$OPTS$NL}$1" ;;
  esac
    shift
done

OPTS="$LEADING_OPTS$NL$DEFAULT_OPTS$NL$OPTS"

SOURCEDIRS=$(ls -d $(find ${DIRS:-$PDIR} -name .git | sed -u 's|/\.git$||') 2>/dev/null | sed -u 's|^./||')

#echo "OPTS='$OPTS'" 1>&2

set -f
set -- $OPTS
set +f 

#exec find $SOURCEDIRS $OPTS -type f -name CMakeLists.txt -exec dirname {} \;
exec find $SOURCEDIRS "$@"
