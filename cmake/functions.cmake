function(DUMP VAR)
  string(REGEX REPLACE "[;\n]" " " A "${ARGV}")
  message("\nVariable dump of: ${A}\n")

  foreach(VAR ${ARGV})
    message("  ${VAR} = ${${VAR}}")
  endforeach(VAR ${ARGV})
  message("")
endfunction(DUMP VAR)

function(CANONICALIZE OUTPUT_VAR STR)
  string(REGEX REPLACE "^-W" "WARN_" TMP_STR "${STR}")

  string(REGEX REPLACE "-" "_" TMP_STR "${TMP_STR}")
  string(TOUPPER "${TMP_STR}" TMP_STR)

  set("${OUTPUT_VAR}" "${TMP_STR}" PARENT_SCOPE)
endfunction(CANONICALIZE OUTPUT_VAR STR)
