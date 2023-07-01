#define _GNU_SOURCE
#include <ctype.h>
#include <string.h>

int
strverscmp(const char* a0, const char* b0) {
  const unsigned char* a = (const void*)a0;
  const unsigned char* b = (const void*)b0;
  size_t i, dp, j;
  int z = 1;
  for(dp = i = 0; a[i] == b[i]; i++) {
    int c = a[i];
    if(!c)
      return 0;
    if(!isdigit(c))
      dp = i + 1, z = 1;
    else if(c != '0')
      z = 0;
  }
  if(a[dp] != '0' && b[dp] != '0') {
    for(j = i; isdigit(a[j]); j++)
      if(!isdigit(b[j]))
        return 1;
    if(isdigit(b[j]))
      return -1;
  } else if(z && dp < i && (isdigit(a[i]) || isdigit(b[i]))) {
    return (unsigned char)(a[i] - '0') - (unsigned char)(b[i] - '0');
  }
  return a[i] - b[i];
}
