#!/bin/sh

ME=${0}
MYNAME=`basename "${ME%.sh}"`

if ! type prettier 2>/dev/null >/dev/null; then
  for DIR in $HOME/.nvm/versions/node/*/bin; do
    PATH="$PATH:$DIR"
  done
fi

prettier() {
 ( set -- ${PRETTIER:-prettier} \
    $OPTS \
    --parser ${PARSER:-babel} \
    --jsx-single-quote \
    --trailing-comma none \
    --write \
    --print-width ${WIDTH:-120} \
    --semi \
    --bracket-spacing \
    ${CONFIG:+--config
"$CONFIG"} \
    --no-insert-pragma \
    "$@"; ${DEBUG:-false} && echo "$@" 1>&2; command "$@" 2>&1 ##| grep -v 'ExperimentalWarning:'
 exit $?)
}

main() {
  EXPR='1 { /@format/ { N; /\n$/ { d } } }'
  for KW in "if" "for" "do" "while" "catch"; do
    EXPR="$EXPR; s|\s${KW}\s*(| ${KW}(|"
    EXPR="$EXPR; s|^${KW}\s*(|${KW}(|"
  done
  EXPR="$EXPR; /($/ { N; s|(\n\s*|(| }"
  EXPR="$EXPR; /([^,]*,$/ { N; s|^\(\s*[^ ]*([^,]*,\)\n\s*{|\1 {| }"
  EXPR="$EXPR; /^\s*[^ ]\+:$/ { N; s|^\(\s*[^ ]\+:\)\n\s*|\1 | }"
  EXPR="$EXPR; /^\s*},$/  { N; s|^\(\s*},\)\n\s*\[|\1 [| }"
  #EXPR="$EXPR; /^\s*},$/ { N; s|},\n\s*|}, |; N; s|\n\s*);$|);| }"
  #EXPR="$EXPR; /($/ { N; s|(\n\s*|(| }"
  #EXPR="$EXPR; /[^ ],$/ { N; s|,\n *{$|, {|g ; s|,\n\s*\([^\n]*\);$|, \1;|g }"

  SEP=${IFS%"${IFS#?}"}

  while [ $# -gt 0 ]; do
    case "$1" in
      -*) OPTS="${OPTS:+$OPTS$SEP}$1"; shift ;;
      *) break ;;
    esac
  done

  [ -f .prettierrc ] && CONFIG=.prettierrc

  if [ $# -le 0 ]; then
    set -- $(find . -maxdepth 1 -type f -name "*.js"; find lib -type f -name "*.js")
    set -- "${@#./}"
    set -- $(ls -td -- "$@")
  fi

  for SOURCE; do
    case "$SOURCE" in
      *.es5.js) continue ;;
    esac
    ARG=${SOURCE//"["/"\\["}
    ARG=${ARG//"]"/"\\]"}
   ( trap 'rm -f "$TMPFILE" "$DIFFFILE"' EXIT
   DIR=`dirname "$ARG"`
    TMPFILE=`mktemp --tmpdir "$MYNAME-XXXXXX.tmp"`
    DIFFFILE=`mktemp --tmpdir "$MYNAME-XXXXXX.diff"`
    echo "Processing ${SOURCE} ..." 1>&2
    case "$SOURCE" in
      *.css) PARSER="css" ;;
      *) unset PARSER ;;
    esac
    prettier <"$ARG" >"$TMPFILE"; R=$?

    if [ $R != 0 ]; then
      cat "$TMPFILE" 1>&2 
      exit 1
     fi
    (: set -x; sed -i  "$EXPR" "$TMPFILE") &&
    {  diff -u "$ARG" "$TMPFILE" | sed "s|$TMPFILE|$ARG|g; s|$ARG|${ARG##*/}|" | tee "$DIFFFILE" |  diffstat |sed "1! d; /0 files/d"  ;  (cd "$DIR" && patch -p0 ) <"$DIFFFILE" && rm -f "$TMPFILE" || mv -vf "$TMPFILE" "$ARG"; }) || return $?
   done
}

main "$@"

