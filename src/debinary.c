// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv) {

  if (argc < 2) {
    printf("usage: %s binaryFile [targetFile]\n", argv[0]);
    return -1;
  }

  FILE *in;
  char array[32];
  char byte;
  unsigned int bla;
  int i = 0, x, y, z;
  in = fopen(argv[1], "r");
  if (argc == 3) {
    FILE *out;
    out = fopen(argv[2], "w");
    do {
      z = fread(array, sizeof(byte), 32, in);
      for (y = 0; y < z; y++) {
        byte = array[y];
        for (x = 7; x >= 0; x--) {
          fprintf(out, "%d", (byte >> x) & 1);
          if (y % 4 == 0 && x == 3)
            fprintf(out, " ");
          if (y % 4 == 1 && x == 4)
            fprintf(out, " ");
          if (y % 4 == 2 && x == 5)
            fprintf(out, " ");
          if (y % 4 == 3 && x == 7)
            fprintf(out, " ");
          if (y % 4 == 3 && x == 0)
            fprintf(out, " - ");
        }
        bla = byte & 0xff;
        if (++i % 8 == 0)
          fprintf(out, "\n");
      }
    } while (z == 32);
    fprintf(out, "\n");
  } else {
    do {
      z = fread(array, sizeof(byte), 32, in);
      for (y = 0; y < z; y++) {
        byte = array[y];
        for (x = 7; x >= 0; x--) {
          printf("%d", (byte >> x) & 1);
          if (y % 4 == 0 && x == 3)
            printf(" ");
          if (y % 4 == 1 && x == 4)
            printf(" ");
          if (y % 4 == 2 && x == 5)
            printf(" ");
          if (y % 4 == 3 && x == 7)
            printf(" ");
          if (y % 4 == 3 && x == 0)
            printf(" - ");
        }
        bla = byte & 0xff;
        if (++i % 8 == 0)
          printf("\n");
      }
    } while (z == 32);
    printf("\n");
  }
  fclose(in);

  return 0;
}
