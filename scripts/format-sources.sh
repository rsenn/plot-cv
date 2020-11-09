#!/bin/sh
##
NL="
"

fs_time() {
  (while :; do case "$1" in
      -t|--time-style|-s) TIME_STYLE="$2"
        shift 2 ;;
      -[ts]=*|--time-style=*) TIME_STYLE="${1#*=}"
        shift ;;
      -[ts]*) TIME_STYLE="${1#-[ts]}"
        shift ;;
      *) break ;;
    esac; done
  fs_lscol 6 "$@")
}


main() {
 (IFS="$NL"

    if [ $# -le 0 ]; then
    set -- ./src ./quickjs
  fi

  while [ $# -gt 0 ]; do
    case "$1" in
      --width|-w) WIDTH=$2; shift 2 ;;
      --width=*|-w=*) WIDTH=${1#*=}; shift ;; 
      -w*) WIDTH=${1#-w}; shift ;;
      *) break ;;
    esac
  done

  ARGS="$*"
 # set -- 
  # for ARG in $ARGS; do
  # 	if [ -d "$ARG" ]; then
  #     set -- "$@"  `find "$ARG" -name '*.[ch]' -or -name '*.[ch]pp'`
  #   else
  # 		set -- "$@"  "$ARG"
  #   fi
  # done

  dump() { (OUT=; for VAR; do eval "OUT=\${OUT:+\$OUT }\$VAR=\$$VAR"; done; echo "$OUT")  1>&2;  }

  list_cmd() {  set -- -f'!d' -i'*.'{h,hpp,c,cpp} "$@"; list-r -c -n -l "$@"; }

  trap 'IFS=$NL; : ls -la  -- $TEMPFILES; rm -f -- $TEMPFILES 1>&2' EXIT

  temp_file() { FILENAME="${2:-${0#-}}-$(( $$ + ${RANDOM:-0} )).tmp"; TEMPFILES="${TEMPFILES:+$TEMPFILES$NL}$FILENAME"; eval "$1=\$FILENAME"; }

  LIST=`: set -x; list_cmd "$@"`

  FILES=`set -- $LIST; echo "${*##* }"`
  (set -- $LIST; echo "Got $# files." 1>&2)

  (set -x; clang-format -style=file  ${WIDTH:+-style="{ColumnLimit:$WIDTH}"} -i $FILES)

  temp_file A

  MATCH=""
  echo "$LIST" >"$A" 
  { IFS=" "; while read  -r  CRC32 MODE N USERID GROUPID SIZE TIME FILE; do
    #dump CRC32 MODE N USERID GROUPID SIZE TIME FILE
    MATCH="${MATCH:+$MATCH\\|}^${CRC32} .* ${FILE}\$"
  done; } <"$A"

  IFS="$NL"; 
  temp_file B 

  #echo "Match: ${MATCH}" 1>&2
  list_cmd "$@"  >"$B"

  { IFS=" "; while read  -r  CRC32 MODE N USERID GROUPID SIZE TIME FILE; do
   (OTHER=$(grep " $FILE\$" "$A")
    read -r OTHER_{CRC32,MODE,N,USERID,GROUPID,SIZE,TIME,FILE} <<<"$OTHER"

  if [ "$CRC32" != "$OTHER_CRC32" -o "$TIME" != "$OTHER_TIME" ]; then
    dump CRC32 MODE N USERID GROUPID SIZE TIME FILE
    dump OTHER_{CRC32,MODE,N,USERID,GROUPID,SIZE,TIME,FILE}

    echo 1>&2
  fi )
    
  done; } <"$B"


#  grep  "$MATCH" "$L" >"$A"


  (: set -x; diff -U0 "$A" "$B" | grep "^[-+][^-+]" | sort -t' ' -k8 )
)
}



      



  



  # for FILE; do
  #   t=`fs_time "$FILE"`
  #   m=`fs_time "$FILE"`
  # done

main "$@"
  