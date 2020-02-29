#!/bin/sh

if ! type prettier 2>/dev/null >/dev/null; then
  for DIR in $HOME/.nvm/versions/node/*/bin; do
    PATH="$PATH:$DIR"
  done
fi

prettier() {
  command prettier --trailing-comma none --write  --print-width ${WIDTH:-120} --semi --bracket-spacing --no-insert-pragma "$@"
}
EXPR='1 { /@format/ { N; /\n$/ { d } } }'
for KW in "if" "for" "do" "while" "catch"; do
EXPR="$EXPR; s|\s${KW}\s*(| ${KW}(|"
done

for SOURCE in  ${@:-$(find components utils stores pages -name "*.js")}; do
  ARG=${SOURCE//"["/"\\["}
  ARG=${ARG//"]"/"\\]"}
  prettier "$ARG"
   sed -i  "$EXPR" "$SOURCE"
 done


