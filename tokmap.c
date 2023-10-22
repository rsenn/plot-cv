#include <stdio.h>

#define TT_CUSTOM 1000

#define TT_LAND TT_CUSTOM + 0
#define TT_LOR TT_CUSTOM + 1
#define TT_LTE TT_CUSTOM + 2
#define TT_GTE TT_CUSTOM + 3
#define TT_SHL TT_CUSTOM + 4
#define TT_SHR TT_CUSTOM + 5
#define TT_EQ TT_CUSTOM + 6
#define TT_NEQ TT_CUSTOM + 7
#define TT_LT TT_CUSTOM + 8
#define TT_GT TT_CUSTOM + 9
#define TT_BAND TT_CUSTOM + 10
#define TT_BOR TT_CUSTOM + 11
#define TT_XOR TT_CUSTOM + 12
#define TT_NEG TT_CUSTOM + 13
#define TT_PLUS TT_CUSTOM + 14
#define TT_MINUS TT_CUSTOM + 15
#define TT_MUL TT_CUSTOM + 16
#define TT_DIV TT_CUSTOM + 17
#define TT_MOD TT_CUSTOM + 18
#define TT_LPAREN TT_CUSTOM + 19
#define TT_RPAREN TT_CUSTOM + 20
#define TT_LNOT TT_CUSTOM + 21

#define TTINT(X) (X - TT_CUSTOM)
#define TTENT(X, Y) [TTINT(X)] = Y

static const int bplist[] = {
  TTENT(TT_LOR, 1 << 4),
  TTENT(TT_LAND, 1 << 5),
  TTENT(TT_BOR, 1 << 6),
  TTENT(TT_XOR, 1 << 7),
  TTENT(TT_BAND, 1 << 8),
  TTENT(TT_EQ, 1 << 9),
  TTENT(TT_NEQ, 1 << 9),
  TTENT(TT_LTE, 1 << 10),
  TTENT(TT_GTE, 1 << 10),
  TTENT(TT_LT, 1 << 10),
  TTENT(TT_GT, 1 << 10),
  TTENT(TT_SHL, 1 << 11),
  TTENT(TT_SHR, 1 << 11),
  TTENT(TT_PLUS, 1 << 12),
  TTENT(TT_MINUS, 1 << 12),
  TTENT(TT_MUL, 1 << 13),
  TTENT(TT_DIV, 1 << 13),
  TTENT(TT_MOD, 1 << 13),
  TTENT(TT_NEG, 1 << 14),
  TTENT(TT_LNOT, 1 << 14),
  TTENT(TT_LPAREN, 1 << 15),
  TTENT(TT_RPAREN, 0),
};

int
main(int argc, char* argv[]) {
  FILE* f = fopen("bplist.bin", "w+b");

  fwrite(bplist, sizeof(bplist), 1, f);
  fclose(f);
}
