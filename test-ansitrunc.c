#include <stdio.h>
#include <string.h>

#define is_alphanumeric_char(c) ((c) >= 'A' && (c) <= 'Z') || ((c) >= 'a' && (c) <= 'z')
#define is_control_char(c) \
  ((c) == '\b' || (c) == '\f' || (c) == '\n' || (c) == '\r' || (c) == '\t' || (c) == '\v')

static inline int
is_escape_char(char c) {
  return is_control_char(c) || c == '\\' || c == '\'';
}

static size_t
ansi_skip(const char* str, size_t len) {
  size_t pos = 0;
  if(str[pos] == 0x1b) {
    if(++pos < len && str[pos] == '[') {
      while(++pos < len)
        if(is_alphanumeric_char(str[pos]))
          break;
      if(++pos < len && str[pos] == '~')
        ++pos;
      return pos;
    }
  }
  return 0;
}

static size_t
ansi_truncate(const char* str, size_t len, size_t limit) {
  size_t i, n = 0, p;
  for(i = 0; i < len;) {
    if((p = ansi_skip(&str[i], len - i)) > 0) {
      i += p;
      continue;
    }
    n += is_escape_char(str[i]) ? 2 : 1;
    if(n > limit)
      break;
    i++;
  }
  return i;
}

int
main() {
  const char* s = "/x86_64-linux-gnu/bits/types/sigevent_t.h:28:5)";
  size_t len = strlen(s);

  printf("%zu\n", ansi_truncate(s, len, 24));
}
