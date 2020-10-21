#!/bin/sh

IFS="
"

set -- ../*/eagle/{lbr/*.lbr,*.{brd,sch}}
set -- "${@%/*}"
set -- $(echo "$*" | sort -u)

echo "$*"

mkdir -m 1777 -p tmp

set -- data=RW "$@"

IFS=":"
OPTS=hide_meta_files

fusermount -z -u tmp >&/dev/null
set -x
exec unionfs-fuse ${OPTS:+-o} ${OPTS:+"$OPTS"} "$*" tmp
