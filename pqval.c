#include <stdio.h>
#include <postgresql/libpq-fe.h>

int
main(int argc, char* argv[]) {
  printf("pg_char_to_encoding(\"utf8\") = %d\n", pg_char_to_encoding("utf8"));
  printf("pg_char_to_encoding(\"UTF8\") = %d\n", pg_char_to_encoding("UTF8"));
  printf("pg_char_to_encoding(\"utf-8\") = %d\n", pg_char_to_encoding("utf-8"));
  printf("pg_char_to_encoding(\"UTF-8\") = %d\n", pg_char_to_encoding("UTF-8"));

  return 0;
}
