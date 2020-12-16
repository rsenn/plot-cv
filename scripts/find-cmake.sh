#!/bin/sh

exec find ${@:-*} "(" -iname CMakeLists.txt -or -iname "*.cmake" -or -iwholename "*/cmake/*" ")" -and -not -wholename "*build/*"
