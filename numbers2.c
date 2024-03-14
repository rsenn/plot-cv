#include <stdio.h>  /* Print functions */
#include <ctype.h>  /* Character identification/conversion */
#include <stdlib.h> /* strtoul */
#include <limits.h>
#include <errno.h>
#include <string.h>

/* Color codes to highlight output */
#define GREEN "\x1b[1;32m"
#define NC "\x1b[0m"

/* Forward declaration of helper functions */
int input_number(unsigned int*);
void dissect_number(char, unsigned int);
void print_number(char, unsigned int, char);
const char* format_name(char);
int format_base(char);

/* Example of inputting a number and then output it in different numeral systems
 */

int
main() {

  for(;;) {
    unsigned int number = 0;
    char format = 'd';

    printf("Prefix: 0b = binary (01), 0o = octal (0-7), 0x = hex (0-9, A-F)\n");
    printf("Enter a number: ");

    if(!(format = input_number(&number))) {
      if(errno) {
        printf("Invalid number: %s\n", strerror(errno));
        return 1;
      }
      return 0;
    }

    printf("Format: %s\n\n", format_name(format));

    dissect_number(format, number);
    printf("\n");

    printf("Decimal: " GREEN "%u" NC "\n", number);

    printf(" Binary: 0b" GREEN);
    print_number('b', number, 0);
    printf(NC "\n");

    printf("  Octal: 0o" GREEN "%o" NC "\n", number);

    printf("    Hex: 0x" GREEN "%X" NC "\n\n", number);
  }

  return 0;
}

/* Read in a number, followed by pressing ENTER */
int
input_number(unsigned int* num) {
  char buffer[1024], fmt[] = {'%', 'd'};
  int i;

  /* Get a line of text */
  if(!fgets(buffer, sizeof(buffer), stdin) || buffer[0] == '\n')
    return 0;

  /* Iterate through the characters of a line */
  for(i = 0; i < sizeof(buffer); i++) {
    const char c = buffer[i];

    if(isdigit(c)) {
      /* If a digit other than zero is entered, then the number begins */
      if(c != '0')
        break;

      /* Leading zero means octal at first */
      fmt[1] = 'o';
      continue;
    }

    /* Alphanumeric character (letter) is format designator */
    if(isalpha(c)) {
      fmt[1] = c;
      i++;
      break;
    }
  }

  char format = tolower(fmt[1]);
  unsigned long n;
  char* end;

  n = strtoul(&buffer[i], &end, format_base(format));

  while(isspace(*end))
    ++end;

  if(*end != '\0' || n == ULONG_MAX || errno == ERANGE || n > UINT_MAX) {
    errno = ERANGE;
    return 0;
  }

  *num = n;

  return format;
}

/* Print out a number in the format specified */
void
print_number(char format, unsigned int num, char pad) {
  char fmt[16] = {'%', format, '\0'};

  switch(format) {
    case 'b': {
      int start, len, i;

      /* loop through the bits */
      for(i = 31, start = 0, len = 0; i >= 0; i--)
        /* start to print only when leading zeroes are skipped */
        if(start || (start = (num >> i) & 1))
          len++;

      if(len < pad)
        for(i = len; i < pad; i++)
          putc('0', stdout);

      /* loop through the bits */
      for(i = 31, start = 0; i >= 0; i--)
        /* start to print only when leading zeroes are skipped */
        if(start || (start = (num >> i) & 1))
          putc(((num >> i) & 1) + '0', stdout);

      break;
    }

    case 'x': {
      sprintf(fmt, pad ? "%%0%uX" : "%%X", pad);
      printf(fmt, num);
      break;
    }

    case 'o': {
      if(pad)
        sprintf(fmt, "%%0%ulo", pad);
      printf(fmt, (unsigned long)num);
      break;
    }

    default: {
      if(pad)
        sprintf(fmt, "%%0%ud", pad);
      printf(fmt, num);
      break;
    }
  }
}

/* Print out a number in the format specified */
void
dissect_number(char format, unsigned int num) {
  int start = 0, len = 0;

  switch(format) {
    case 'b':

      /* loop through the bits */
      for(int i = 31; i >= 0; i--) {
        const char d = (num >> i) & 1;

        /* start to print only when leading zeroes are skipped */
        if(start || (start = d ? i : 0))
          len++;
      }

      /* loop through the bits */
      for(int i = 31; i >= 0; i--) {
        const char d = (num >> i) & 1;

        /* start to print only when leading zeroes are skipped */
        if(i <= start) {
          // printf(" %u x %10u (2 ^ %u)\n", d, 1 << i, i);
          printf(" %u x %10u (0b", d, 1 << i);
          print_number('b', 1 << i, len);
          printf(" = 2 ^ %u)\n", i);
        }
      }
      break;

    case 'o':

      /* loop through the triplets */
      for(int i = 10; i >= 0; i--) {
        const char d = (num >> (i * 3)) & 07;

        /* start to print only when leading zeroes are skipped */
        if(start || (start = d ? i : 0))
          // printf(" %o x %10u (0o%011o)\n", d, 1 << (i * 3), 1 << (i * 3));
          printf(" %o x %10u (0o%011o = 3 ^ %u)\n",
                 d,
                 1 << (i * 3),
                 1 << (i * 3),
                 i);
      }

      break;

    case 'x':
      /* loop through the nibbles */
      for(int i = 7; i >= 0; i--) {
        const char d = (num >> (i * 4)) & 0xf;

        /* start to print only when leading zeroes are skipped */
        if(start || (start = d ? i : 0))
          // printf(" %X: %2u x %10u (0x%08X)\n", d, d, 1 << (i * 4), 1 << (i *
          // 4));
          printf(" %X: %2u x %10u (16 ^ %u)\n", d, d, 1 << (i * 4), i);
      }

      break;

    default:
      /* loop through the digits */
      for(int i = 9, mul = 1e9; i >= 0; i--, mul /= 10) {
        const char d = (num / mul) % 10;

        /* start to print only when leading zeroes are skipped */
        if(start || (start = d))
          printf(" %u x %10u (10 ^ %u)\n", d, mul, i);
      }

      break;
  }
}

const char*
format_name(char fmt) {
  switch(fmt) {
    case 'x': return "hexadecimal";
    case 'b': return "binary";
    case 'o': return "octal";
    default: return "decimal";
  }
}

int
format_base(char fmt) {
  switch(fmt) {
    case 'x': return 16;
    case 'b': return 2;
    case 'o': return 8;
    default: return 10;
  }
}
