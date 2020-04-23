#!/bin/sh

if ! type prettier 2>/dev/null >/dev/null; then
  for DIR in $HOME/.nvm/versions/node/*/bin; do
    PATH="$PATH:$DIR"
  done
fi

prettier() {
  command prettier \
    $OPTS \
    --jsx-single-quote \
    --trailing-comma none \
    --write \
    --print-width ${WIDTH:-120} \
    --semi \
    --bracket-spacing \
    --no-insert-pragma \
    "$@"
}
EXPR='1 { /@format/ { N; /\n$/ { d } } }'
for KW in "if" "for" "do" "while" "catch"; do
  EXPR="$EXPR; s|\s${KW}\s*(| ${KW}(|"
  EXPR="$EXPR; s|^${KW}\s*(|${KW}(|"
done

while [ $# -gt 0 ]; do
  case "$1" in
    -*) OPTS="${OPTS:+$OPTS
}$1"; shift ;;
    *) break ;;
  esac
done

for SOURCE in  ${@:-$(find components utils stores pages -name "*.js")}; do
  case "$SOURCE" in
    *.es5.js) continue ;;
  esac
  ARG=${SOURCE//"["/"\\["}
  ARG=${ARG//"]"/"\\]"}
  prettier "$ARG"
  (: set -x; sed -i  "$EXPR" "$SOURCE")
 done


