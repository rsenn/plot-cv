#!/bin/sh
##
if [ $# -le 0 ]; then
  set -- ./src ./quickjs
fi

builddir=`ls -td build/* | head -n1`
echo "Build dir is ${builddir}" 1>&2

export builddir

if [ -e "$builddir/build.ninja" ]; then
  CMD="ninja -C ${builddir}"
else 
  CMD="make -C ${builddir}"
fi


set -- iwatch -v -c "$CMD" -e close_write -t '.*\.(h|hpp|c|cpp)$$' -x '.*/build/.*' -X'./(\.|tmp|static|lib|.git).*' -r "$@"

exec "$@" 