#include <stdio.h>
#include <wchar.h>
#include <windows.h>

int main() {
  printf("sizeof(wchar_t) = %u\n", sizeof(wchar_t));
  printf("sizeof(TCHAR) = %u\n", sizeof(TCHAR));
}
