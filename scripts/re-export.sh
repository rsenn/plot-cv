#!/bin/sh

SEP="
"
pushv() { 
      eval "shift;$1=\"\${$1+\"\$$1\${SEP%\"\${SEP#?}\"}\"}\$*\""
}
pushv_unique () 
{ 
    __VAR=$1 __ARG=
    shift;
    for __ARG ; do
      eval "(IFS=\"\$SEP\"; ! isin \$__ARG \${$__VAR})" && pushv "$__VAR" "$__ARG" || return 1;
    done
}
isin() 
{ 
    ( needle="$1";
    while [ "$#" -gt 1 ]; do
        shift;
        test "$needle" = "$1" && exit 0;
    done;
    exit 1 )
}
implode () 
{ 
    ( unset DATA SEPARATOR;
    SEPARATOR="$1";
    shift;
    CMD='DATA="${DATA+$DATA$SEPARATOR}$ITEM"';
    if [ $# -gt 0 ]; then
        CMD="for ITEM; do $CMD; done";
    else
        CMD="while read -r ITEM; do $CMD; done";
    fi;
    eval "$CMD";
    echo "$DATA" )
}

unset VARS
    
grep -r -H '^\s*export' "$@" | { IFS=":"; while read -r FILE CODE; do 
  #echo "FILE='$FILE' CODE='$CODE'" 1>&2
  CODE=${CODE#*export?}
  CODE=${CODE#"const "}
  CODE=${CODE#"let "}
  CODE=${CODE#"class "}
  CODE=${CODE#"var "}
  CODE=${CODE#"function "}

  case "$CODE" in
    "{"*"}"*)
     NAME=$(IFS="${SEP}{}, "; set -- ${CODE%%"}"*}; echo "$*") ;;
     *)   NAME=${CODE%%[!A-Za-z_0-9]*} ;;
  esac

  VAR=$(echo "$FILE" |sed 's,[-./],_,g')

  pushv "${VAR}_EXPORTS" "$NAME" 
  pushv_unique VARS  "${VAR}" 
  eval  "${VAR}_FILENAME=\$FILE" 

done
echo $VARS 1>&2
IFS="$SEP"
for V in $VARS; do
  VCMD="IDS=\${${V}_EXPORTS} FILE=\${${V}_FILENAME}"
  eval "$VCMD"
  #echo "V='$V' IDS='$IDS' FILE='$FILE'" 1>&2
  case "$FILE" in
    ./*) ;;
    *) FILE="./$FILE" ;;
  esac
  IDS=$(echo "$IDS" | LC_ALL=C sort -u)
  set "${CMD-export} { $(implode ', ' $IDS) } from '$FILE'"
  echo "$@"
done


 }
