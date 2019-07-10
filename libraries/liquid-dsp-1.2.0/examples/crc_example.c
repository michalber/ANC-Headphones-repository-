//
// crc_example.c
//
// Cyclic redundancy check (CRC) example.  This example demonstrates
// how a CRC can be used to validate data received through un-reliable
// means (e.g. a noisy channel).  A CRC is, in essence, a strong
// algebraic error detection code that computes a key on a block of data
// using base-2 polynomials.
// SEE ALSO: checksum_example.c
//           fec_example.c
//

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include "liquid.h"

// print usage/help message
void usage()
{
    printf("crc_example [options]\n");
    printf("  u/h   : print usage\n");
    printf("  n     : input data size (number of uncoded bytes)\n");
    printf("  v     : checking scheme, (crc32 default):\n");
    liquid_print_crc_schemes();
}


int main(int argc, char*argv[])
{
    // options
    unsigned int n = 32;                // data length (bytes)
    crc_scheme check = LIQUID_CRC_32;   // error-detection scheme

    int dopt;
    while((dopt = getopt(argc,argv,"uhn:v:")) != EOF){
        switch (dopt) {
        case 'h':
        case 'u': usage(); return 0;
        case 'n': n = atoi(optarg); break;
        case 'v':
            check = liquid_getopt_str2crc(optarg);
            if (check == LIQUID_CRC_UNKNOWN) {
                fprintf(stderr,"error: unknown/unsupported error-detection scheme \"%s\"\n\n",optarg);
                exit(1);
            }
            break;
        default:
            exit(1);
        }
    }

    // validate input

    unsigned int i;

    // initialize data arra
    unsigned char data[n];
    for (i=0; i<n; i++)
        data[i] = rand() % 256;

    // compute key on original data
    unsigned int key = crc_generate_key(check, data, n);
    printf("key: 0x%.8x\n", (unsigned int) key);

    printf("testing uncorrupted data... ");
    if (crc_validate_message(check, data, n, key))
        printf("  pass\n");
    else
        printf("  FAIL\n");

    printf("testing corrupted data...   ");
    data[0]++;
    if (crc_validate_message(check, data, n, key))
        printf("  pass\n");
    else
        printf("  FAIL\n");

    printf("done.\n");
    return 0;
}
