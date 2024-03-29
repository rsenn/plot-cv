#include <stdio.h> /* Print functions */
#include <ctype.h> /* Character identification/conversion */

/* Color codes to highlight output */
#define GREEN "\x1b[1;32m"
#define NC "\x1b[0m"

/* Forward declaration of helper functions */
int input_number(unsigned int*);
int print_binary(unsigned int);

/* Example of inputting a number and then output it in different numeral systems */

int
main() {

  for(;;) {
    unsigned int number;
    char format;

    printf("Prefix: 0b = binary (01), 0o = octal (0-7), 0x = hex (0-9, A-F)\n");
    printf("Enter a number: ");

    if(!(format = input_number(&number))) {
      printf("Invalid number!\n");
      return 1;
    }

    printf("Format: %c\n", format);
    printf("\n");

    printf("Decimal: " GREEN "%u" NC "\n", number);

    printf(" Binary: 0b" GREEN);
    print_binary(number);
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
  if(!fgets(buffer, sizeof(buffer), stdin))
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

  /* if the format is binary, we must scan it ourselves */
  if(tolower(fmt[1]) == 'b') {
    *num = 0;

    for(int j = 0; buffer[i + j] && j < 32; j++) {
      switch(buffer[i + j]) {
        case '1':
          *num <<= 1;
          *num |= 1;
          break;
        case '0': *num <<= 1; break;
        default: return fmt[1];
      }
    }

    return fmt[1];
  }

  /* other formats scanned by library function */
  return sscanf(&buffer[i], fmt, num) ? fmt[1] : 0;
}

/* Print out a number as binary */
int
print_binary(unsigned int num) {
  int start = 0;

  /* loop through the bits */
  for(int i = 31; i >= 0; i--)
    /* start to print only when leading zeroes are skipped */
    if(start || (start = (num >> i) & 1))
      putc(((num >> i) & 1) + '0', stdout);
}
