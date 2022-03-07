#!/bin/sh
#
#  sed -e '/^import\s/ { :lp; /;\s*$/! { N;  b lp }; s,\s*\n\s*, ,g }' 

collapse_imports() {
  sed -e "/^import\\s/ { :lp; /;\\s*\$/! { N;  b lp }; s,\\s*\\n\\s*, ,g }" "$@"
}

collapse_imports "$@"
