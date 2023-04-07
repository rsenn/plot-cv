#!/bin/sh
ME=$0
MYNAME=`basename "${ME%.sh}"`
MYDIR=`dirname "$ME"`
IFS="
"
pushv () 
{ 
    eval "shift;$1=\"\${$1+\"\$$1\${IFS%\"\${IFS#?}\"}\"}\$*\""
}

usage() {
  echo "Usage: $MYNAME [options] <build-directories...>" 1>&2
  exit 1
}

iwatch() {
  while [ $# -gt 0 ]; do 
    case "$1" in
      -j*) NJOBS=${1#-j}; shift ;; -j) NJOBS=${2}; shift 2 ;;
      -t*) TOOL=${1#-t}; shift ;; -t) TOOL=${2}; shift 2 ;;
      -r|--recursive) RECURSIVE=true; shift ;; 
      -s) pushv SOURCEDIRS "${2}"; shift 2 ;;
      -s*) pushv SOURCEDIRS "${1#-s}"; shift ;; 
      -b) pushv BUILDDIRS "${2}"; shift 2 ;;
      -b*) pushv BUILDDIRS "${1#-b}"; shift ;; 
      *) 
        if [ -z "$BUILDDIRS" -a -n "$1" -a -d "$1" ]; then
          BUILDDIRS="${BUILDDIRS+$BUILDDIRS$IFS}$1"
        elif [ -z "$TARGET" ]; then
          TARGET="$1"
        fi
        shift  ;;
    esac
  done
  echo "SOURCEDIRS:" $SOURCEDIRS 1>&2
  echo "BUILDDIRS:" $BUILDDIRS 1>&2
  
  if [ -z "$BUILDDIRS" ]; then
    usage
  fi

  if [ -z "$SOURCEDIRS" ]; then
    set -- 
    for DIR in $BUILDDIRS; do
      set -- "$@" "${DIR%%/build/*}"
    done
    SOURCEDIRS=$*
  fi

  : ${builddir=`ls -td $BUILDDIRS | head -n1`}
  echo "Build dir:" $builddir 1>&2
  : ${SOURCEDIRS=$(ls -td $builddir | sed 's,/build/.*,,')}
  #sourcedir=${builddir%%/build/*}
  sourcedir=$(set -- $SOURCEDIRS; echo "$1")
  echo "Source dir:" $sourcedir 1>&2

  export builddir sourcedir

  if type samu 1>/dev/null 2>/dev/null; then
    : ${NINJA=samu}
  else
    : ${NINJA=ninja}
  fi
  
  : ${MAKE=make}

  if [ -e "$builddir/build.ninja" ]; then
    CMD="$NINJA -C ${builddir}"
  else 
    CMD="$MAKE -C ${builddir} --no-print-directory"
  fi

  : ${NJOBS=10}

  if [ -n "$TOOL" ]; then
    CMD="$CMD -t $TOOL"
  elif [ -n "$NJOBS" ]; then
    CMD="$CMD -j$NJOBS"
  fi

   if [ -n "$TARGET" ]; then
    CMD="$CMD $TARGET"
  fi
  #CMD="bash -c \"$CMD; echo; echo 'Build done: \$(date)' 1>&2\""

  #EVENTS=modify,create
  EVENTS=close_write 

  set -- iwatch ${RECURSIVE:+-r} -v -c "$CMD" -e "$EVENTS" -t '.*\.(h|hpp|c|cpp|js)$' -x '.*/build/.*' -X'./(\.|tmp|static|.git|CMakeFiles).*' $sourcedir
  for ARG; do 
    case "$ARG" in
      *\ * | *[\$\(\)\|]*) ARG="'$ARG'" ;;
    esac
    case "$ARG" in
      *\ * ) ARG="\\$IFS  $ARG \\$IFS  " ;;
    esac

    OUT="${OUT:+$OUT }$ARG"
  done
  echo "$OUT" 1>&2
  exec "$@"
}

iwatch "$@"
