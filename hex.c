#include <stdio.h>  /* Print functions */
#include <limits.h> /* UINT_MAX constant */
#include <assert.h> /* assertions */

/* Color codes to highlight output */
#define RED "\x1b[1;31m"
#define GREEN "\x1b[1;32m"
#define YELLOW "\x1b[1;33m"
#define NC "\x1b[0m"

/* Example showing how text or a number is represented as hexadecimal bytes */

int
main() {
  /* Text is represented as a string, which is an sequence of characters, each
     one is 1 byte. The end of the string is indicated by a nul-character
     respectively the byte 0x00 */
  const char text[] = "Forever is composed of nows";

  printf("Text to Hex: " GREEN);

  /* We go through each character of the text, as long as the byte is not 0x00,
     and we print the hexadecimal value of the character */
  for(int i = 0; text[i]; i++)
    printf("%s%02X", i > 0 ? " " : "", text[i]);

  printf(NC "\n\n");

  /* This is an array of numbers of 1 byte each.
     For example 0x47 (Decimal: 71) is the character 'G' and 041 (Decimal 65) is
     'A'. End of string, indicated by a nul-character */
  const char hex[] = {
      0x47, 0x72, 0x61, 0x74, 0x69, 0x74, 0x75, 0x64, 0x65, 0x20, 0x74,
      0x75, 0x72, 0x6e, 0x73, 0x20, 0x77, 0x68, 0x61, 0x74, 0x20, 0x77,
      0x65, 0x20, 0x68, 0x61, 0x76, 0x65, 0x20, 0x69, 0x6e, 0x74, 0x6f,
      0x20, 0x65, 0x6e, 0x6f, 0x75, 0x67, 0x68, 0x00,
  };

  /* But this way we interpret the array of numbers as a string and print it
   * out: */
  printf("Hex to text: " GREEN "%s" NC "\n\n", hex);

  /* Let the user enter a decimal number */
  printf("Enter a number (up to %u): ", UINT_MAX);

  /* A 32-bit number (consisting of 4 bytes, each one is 8-bit) */
  unsigned int number, total_sum = 0;

  if(scanf("%u", &number) != 1) {
    fprintf(stderr, "Not a valid number\n");
    return 1;
  }

  /* Print it out in hexadecimal, pad it to 8 digits */
  printf("\nNumber in hexadecimal: " RED "%08X" NC "\n", number);

  /* Loop 8 times because there are 8 hexadecimal digits in the number.
     Starting with the rightmost digit, which is the least significant digit
     (the one with the lowest value) */
  for(int i = 0; i < 8; i++) {
    char shift = i * 4; /* How many bits to shift */

    /* The current digit is obtained by shifting it n bits right and by
     * logically AND it (also called masking) */
    char digit = (number >> shift) & 0x0F;

    /* If the digit is not 0, output its value */
    if(digit)
      printf(RED "%2X" NC
                 " = %3u × %10u (Hex: 0x%08X, Shift left: %2u) = %10u\n",
             digit,
             digit,
             1 << shift,
             1 << shift,
             shift,
             digit << shift);

    /* Add it to total sum */
    total_sum += digit << shift;
  }

  /* total_sum should equal the number now, if not it spits an error message */
  assert(total_sum == number);

  /* Print it out in hexadecimal, pad it to 8 digits */
  printf(
      "--------------------------------------------------------------------\n");
  printf("                                               Total sum: " YELLOW
         "%10u" NC "\n\n",
         total_sum);

  for(char i = 1; i <= 4; i++) {
    int bits = i * 8;
    unsigned long max_value = (1ul << bits) - 1;
    printf("%2u-bit numbers (%u bytes) go from 0 to %10lu (Hex: 0x%lX)\n",
           bits,
           i,
           max_value,
           max_value);
  }

  /* Let the user enter a decimal number */
  printf("\nEnter a hex number (up to 8 digits): ");

  if(scanf("%x", &number) != 1) {
    fprintf(stderr, "Not a valid number\n");
    return 1;
  }

  /* Print it out in decimal, pad it to 8 digits */
  printf("\nNumber in decimal: " YELLOW "%10u" NC "\n", number);
  printf("Number in binary: " YELLOW);

  /* Loop 32 times to show each bit */
  for(int i = 31; i >= 0; i--) {

    /* If the digit is not 0, output its value */
    printf("%d%s", (number >> i) & 1, i > 0 && (i & 0x3) == 0 ? " " : "");
  }
  printf(NC "\n\n");

  /* Loop 8 times because there are 8 hexadecimal digits in the number. */
  for(int i = 7; i >= 0; i--) {
    char shift = i * 4; /* How many bits to shift */

    /* The current digit is obtained by shifting it n bits right and by
     * logically AND it (also called masking) */
    char digit = (number >> shift) & 0x0F;

    /* If the digit is not 0, output its value */
    printf(RED "%2X" NC " = %3u (Binary: 0b%d%d%d%d, Shift left: %2u)\n",
           digit,
           digit,
           (digit >> 3) & 1,
           (digit >> 2) & 1,
           (digit >> 1) & 1,
           (digit)&1,
           shift);

    /* Add it to total sum */
    total_sum += digit << shift;
  }

  return 0;
}
