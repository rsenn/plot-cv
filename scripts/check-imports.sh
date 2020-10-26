#!/bin/sh
#
# sed -n  '/^\s*import/ { s,.*from\s,, ; s|["'\'';]||g; \|/|! { /\.js$/! p }  }' "

check_imports() {
  SCRIPT="${ARG##*/}.sed"
  (IFS="
"
  IMPORTS=$(grep -E "(^\s*import|=.*require[\s\(])" "$1" )
  CODE=$(grep -v -E "(^\s*import|=.*require[\s\(])" "$1" )
  IDENTIFIERS=$(for LINE in $IMPORTS; do
    case "$LINE" in
      --*) continue ;;
      *import*) LINE=${LINE#*import} ;;
    esac
    LINE=${LINE##*\{}
    LINE=${LINE%%\}*}
    LINE=${LINE%%require*}
    LINE=${LINE%%=*}
    LINE=${LINE%% from *}
    test -n "$LINE" &&
    echo "$LINE" | sed "s|[^[:alnum:]_]\+|\n|g"
  done | sort -u)
  test -n "$IDENTIFIERS" &&
  echo "$1:" $IDENTIFIERS 1>&2
  case "$1" in
    *.js) KEYWORD=import ;;
    *.moon) KEYWORD=require ;;
  esac
  OUT=
  for I in $IDENTIFIERS; do
    case "$I" in
      React|Component) continue ;;
      *[!A-Za-z0-9_]*) continue ;;
      -- | from | as | "*") continue ;;
    esac
  
    (echo "$CODE" | tokgrep.sh -q "$I" --) || OUT="${OUT:+$OUT
}s|[, ]\+$I\([, ]\+\)|\1|g"
  done
  test -n "$OUT" &&   
  OUT="/$KEYWORD/ {$OUT
}
/import.*{.*}.*from/ { s|\s*{\s*}\s*| |g ;; s|{,|{| ;; s|\s*,\s*from| from| }
s|^\s*import\s*,\s*|import |
\\|^\s*import \s*from|d"
echo "$OUT"
  ) >"$SCRIPT"
  EXPR=$( OUT=; while read -r  LINE; do OUT="${OUT:+$OUT ;; }$LINE"; done <$SCRIPT; echo "$OUT")
  EXPR="$EXPR; s|\\x01|, |g"
  test -n "$EXPR" &&
  (set -x
  sed -i -e "$EXPR" "$1")
}

for ARG; do
 check_imports "$ARG"
done
