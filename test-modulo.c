#include <stdio.h>
#include <inttypes.h>

const uint64_t usize = 100;
const int64_t size = 100;

void
print_u(uint64_t u) {
  printf("u = %" PRIu64 "\n", u);
}

void
print_i(int64_t i) {
  printf("i = %" PRId64 "\n", i);
}

int
main() {
  print_u(234 % usize);
  print_u(99 % usize);
  print_i(-1);
  print_i(-1 % size);
  print_i(-185);
  print_i(-185 % size);
}
