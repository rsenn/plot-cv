#!/bin/sh

IFS="
"

while [ $# -gt 0 ]; do
  case "$1" in
    -f | -d) ARGS="${ARGS:+$ARGS
}$1"; shift ;;
    *) break ;;
  esac
done

if [ $# -le 0 ]; then
  set -- $( (find ../*/eagle "(" -iname "*.lbr" ")"; find ../*/eagle -maxdepth 1 "(" -iname "*.sch" -or -iname "*.brd" ")" ) |sed 's,/[^/]*$,,' |sort -u)
  #set -- ../*/eagle/{lbr/*.lbr,*.{brd,sch}}
  #set -- "${@%/*}"
  #set -- $(echo "$*" | sort -u)
fi

echo "$*" 1>&2

mkdir -m 1777 -p tmp
if [ ! -d data ]; then
  mkdir -p data
fi

set -- data=RW "$@"

IFS=":"
OPTS=${OPTS:+$OPTS,}hide_meta_files
echo "OPTS='$OPTS'" 1>&2
set -x
fusermount -z -u tmp 2>/dev/null >/dev/null
exec unionfs-fuse ${OPTS:+-o} ${OPTS:+"$OPTS"} $ARGS "$*" tmp 2>&1
