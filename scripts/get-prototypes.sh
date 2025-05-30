#!/bin/sh
ME=`basename "$0" .sh`
THISDIR=`dirname "$0"`
BASEDIR=`cd "$THISDIR"/.. && pwd`
NL="
"

check_support_arg() {
  (CMD="cproto -h 2>&1 | grep -q '^\\s*$1\\s'"
  eval "$CMD") }

get_preprocessor() {
  (set -- $(ls -d /usr/bin/cpp{,-[0-9]*} |sort -fuV )
   eval "echo \$${#}") 2>/dev/null
}

check_exec() {
  (CMD="\"\$@\" 2>&1 1>/dev/null <<<\"\" || echo \"ERROR: \$?\""
  eval "ERROR=\$($CMD)"
  test -z "$ERROR")
}

filter() { 
 (PATTERN="$1"; shift; OUT=; for ARG; do case "$ARG" in 
   $PATTERN) OUT="${OUT:+$OUT${IFS:0:1}}$ARG" ;;
  esac; done; echo "$OUT")
}

read_proto() {
  read -r LINE || return $?
   FN=${LINE%%"("*}
   ARGS=${LINE#$FN}
   FNAME=${FN##*" "}
   TYPE=${FN%"$FNAME"}
   TYPE=${TYPE%" "}
   case "$FNAME" in
     \**)  TYPE="$TYPE*"; FNAME=${FNAME#"*"} ;;
   esac
  #echo "TYPE=$TYPE FNAME=$FNAME ARGS=$ARGS"   1>&7
}

adjust_length() {
  eval "[  \${${1}_MAXLEN} -ge \${#$1} ] 2>/dev/null || ${1}_MAXLEN=\${#$1}"
}

clean_args() {
  old_IFS="$IFS"; IFS=","
  : ${DEPTH:=0}
  ARGS=${1#"("}
  set -- ${ARGS%")"*}
  echo "[$DEPTH] clean_args $*" 1>&7
  [ "${OUT+set}" != set ] && OUT=
  ARG=
  I=0
  while [ $# -gt 0 ]; do 
    ARG="${ARG:+$ARG, }${1# }"; shift

    : $((++I))

   #[ "$REMOVE_NAMES" = "$I" ] && REMOVE_NAMES=true

    P1=${ARG%%")("*}
    P2=${ARG#"$P1"}
    case "$P2" in
      ")("*")"*) echo "[$DEPTH] Ok: $ARG" 1>&7 ;;
      ")("*) echo "[$DEPTH] Again" 1>&7; continue ;;
      *) ;;
    esac
         
    echo "[$DEPTH] ARG='$ARG'" 1>&7

    ARG2=
    case "$ARG" in
      *")("*")"*) 
        ARG1=${ARG%%")("*}
        ARG2=${ARG#$ARG1}
         
        ARG2=$(DEPTH=$((DEPTH+1)) OUT= clean_args "${ARG2#")"}")
        ARG="$ARG1)"
        ;;
    esac
    ARG=${ARG#"("}
    ARG=${ARG#" "}
    ARG=${ARG//" ***"/"*** "}
    ARG=${ARG//" **"/"** "}
    ARG=${ARG//" *"/"* "}
   
    if [ "$REMOVE_NAMES" = true ] || [ -n "$REMOVE_NAMES" -a "$REMOVE_NAMES" -ge "$I" ] 2>/dev/null; then 
      ARG=${ARG%" "[[:alpha:]]*}
      ARG=${ARG%" "}
    fi

    [ "$EMPTY" = true -a -n "$ARG2" ] && ARG2="()"
    ARG=$ARG$ARG2
    #ARG=${ARG//' )'/')'}
     OUT="${OUT:+$OUT, }$ARG"
    ARG=
  done
  echo "[$DEPTH] OUT='$OUT'" 1>&7
  echo "($OUT)"
}

get_prototypes() {
  : ${PAD_ARGS=false}
  : ${CPROTO_ARGS=-Iquickjs}
  while :; do
    case "$1" in
      -[dx] | --debug) DEBUG=true; shift ;;
      -A | --no-pad-args* | -*no*args*) PAD_ARGS=false; shift ;;
      -a | --pad-args* | -*args*) PAD_ARGS=true; shift ;;
      -r=* | --remove*=* | -R=*) REMOVE_NAMES=${1#*=}; shift ;;
      -I* ) CPROTO_ARGS="$CPROTO_ARGS$NL$1" ; shift ;; 
      -r | --remove* | -R) REMOVE_NAMES=true; shift ;;
      -c | --copy* | --xclip*) XCLIP=true; shift ;;
      -E | --ellips* | --empty*) EMPTY=true; shift ;;
      -q | --quiet) QUIET=true; shift ;;
      -e | --expr) EXPR="${EXPR:+$EXPR ;; }$2"; shift 2 ;;
      -e=* | --expr=*) EXPR="${EXPR:+$EXPR ;; }$2"; shift ;;
      -e* ) EXPR="${1#-e}"; shift ;;
      *) break ;;
    esac
  done
 IFS="$NL"
  add_arg() {
    CPROTO_ARGS="$CPROTO_ARGS$NL$*"
   }
  if [ "$DEBUG" = true ]; then
    exec 7>&2
  else
    exec 7>/dev/null
  fi
  if check_support_arg -N; then
    add_arg -N
  fi
  if check_support_arg -n; then
    add_arg -n
  fi
  PP=$(get_preprocessor)
  if [ -x "$PP" ]; then
     check_exec "$PP" -std=c2x &&  
      add_arg "-E" "$PP" ||
      add_arg "-E" "$PP"
  fi
  if [ "$QUIET" = true ]; then
    add_arg -q
    CPROTO_REDIR="2>/dev/null"
  fi
  CPROTO_CMD="cproto \$CPROTO_ARGS -D__restrict= -D__THROW= -D_Noreturn= -D__{value,x,y}= -p \"\$@\" $CPROTO_REDIR | sed \"\\|^/|d ;; s|\b_Bool\b|bool|g ;; $EXPR\""
  if [ "$DEBUG" = true ]; then
    eval "echo \"Command:\" $CPROTO_CMD 1>&2"
  fi
  CPROTO_OUT=`eval "$CPROTO_CMD"`
 
  IFS=" "
  while read_proto; do
    adjust_length TYPE
    adjust_length FNAME
    adjust_length ARGS
  done <<<"$CPROTO_OUT"

 (TEMP=`mktemp`
  trap 'rm -f "$TEMP"' EXIT 
  [ "$PAD_ARGS" = true ] && PAD_A2="-$((FNAME_MAXLEN))"
  while read_proto; do
    set -- "$TYPE" "$FNAME" "$ARGS"
    printf "%-$((TYPE_MAXLEN))s |%${PAD_A2}s|%s\n" "$1" "$2" "$(clean_args "$3");"
  done <<<"$CPROTO_OUT" | 
      sed "s,|,,g" >"$TEMP"
      #sort -t'|' -k2 -f |

  [ "$XCLIP" = true ] && xclip -selection clipboard -in <"$TEMP"
  cat "$TEMP"
  )
}

get_prototypes "$@"
