#!/bin/sh
##
if [ $# -le 0 ]; then
  set -- ./src ./quickjs
fi

: ${builddir=`ls -td build/* | head -n1`}
echo "Build dir is ${builddir}" 1>&2

export builddir

if [ -e "$builddir/build.ninja" ]; then
  CMD="ninja -C ${builddir}"
else 
  CMD="make -C ${builddir}"
fi

#EVENTS=modify,create
EVENTS=close_write 

set -- iwatch -v -c "$CMD" -e "$EVENTS" -t '.*\.(h|hpp|c|cpp)$' -x '.*/build/.*' -X'./(\.|tmp|static|lib|.git).*' -r "$@"
for ARG; do 
  case "$ARG" in
    *\ * | *[\$\(\)\|]*) ARG="'$ARG'" ;;
esac
  case "$ARG" in
    *\ * ) ARG="\\
$ARG \\
" ;;
esac

  OUT="${OUT:+$OUT }$ARG"
done
echo "$OUT" 1>&2
exec "$@"
