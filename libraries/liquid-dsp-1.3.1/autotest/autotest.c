/*
 * Copyright (c) 2007 - 2015 Joseph Gaeddert
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// autotest.c
//
// This file is used in conjunction with autotest_include.h (generated with
// autotest_gen.py) to produce an executable for automatically testing the
// various signal processing algorithms in liquid.

// default include headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include "autotest/autotest.h"

void usage()
{
    // help
    printf("Usage: xautotest [OPTION]\n");
    printf("Execute autotest scripts for liquid-dsp library.\n");
    printf("  -h            display this help and exit\n");
    printf("  -t <id>       run specific test\n");
    printf("  -p <id>       run specific package\n");
    printf("  -r            run all tests, random order\n");
    printf("  -L            lists all scripts\n");
    printf("  -l            lists all packages\n");
    printf("  -x            stop on fail\n");
    printf("  -s <string>   run all tests matching search string\n");
    printf("  -v            verbose\n");
    printf("  -q            quiet\n");
    printf("  -o <filename> output file (json)\n");
}

// define autotest function pointer
typedef void(autotest_function_t) (void);

// define autotest_t
typedef struct {
    unsigned int id;                // test identification
    autotest_function_t * api;      // test function, e.g. autotest_modem()
    const char* name;               // test name
    long unsigned int num_checks;   // number of checks that were run for this test
    long unsigned int num_passed;   // number of checks that passed
    long unsigned int num_failed;   // number of checks that failed
    long unsigned int num_warnings; // number of warnings 
    float percent_passed;           // percent of checks that passed
    int executed;                   // was the test executed?
    int pass;                       // did the test pass? (i.e. no failures)
} autotest_t;

// define package_t
typedef struct {
    unsigned int id;            // package identification
    unsigned int index;         // index of first autotest
    unsigned int num_scripts;   // number of tests in package
    const char* name;           // package name
    int executed;               // were any tests executed?
} package_t;

// include auto-generated autotest header
//
// defines the following symbols:
//   #define AUTOSCRIPT_VERSION
//   #define NUM_AUTOSCRIPTS
//   autotest_t scripts[NUM_AUTOSCRIPTS]
//   #define NUM_PACKAGES
//   struct package_t packages[NUM_PACKAGES]
#include "../autotest_include.h"

// 
// helper functions:
//

// execute a specific test
void execute_autotest(autotest_t * _test, int _verbose);

// execute a specific package
void execute_package(package_t * _p, int _verbose);

// execute a specific package if string matches
void execute_package_search(package_t * _p, char * _str, int _verbose);

// print all autotest results
void print_autotest_results(autotest_t * _test);

// print the results of a particular package
void print_package_results(package_t * _p);

// print all unstable tests (those which failed or gave warnings)
void print_unstable_tests(void);

// print list of tests
void print_test_list(void);

// print list of packages
void print_package_list(void);

// print basic autotest results to stdout
int export_results(char * _filename);

// main function
int main(int argc, char *argv[])
{
    // options
    enum {RUN_ALL,              // run all tests
          RUN_ALL_RANDOM,       // run all tests (random order)
          RUN_SINGLE_TEST,      // run just a single test
          RUN_SINGLE_PACKAGE,   // run just a single package
          RUN_SEARCH
    } mode = RUN_ALL;

    // set defaults
    unsigned int autotest_id        = 0;
    unsigned int package_id         = 0;
    int          verbose            = 1;
    int          stop_on_fail       = 0;
    int          rseed              = 0;
    char         search_string[128] = "";
    char         filename[256]      = "";

    unsigned int i;

    // get input options
    int d;
    while((d = getopt(argc,argv,"ht:p:rLlxs:vqo:")) != EOF){
        switch (d) {
        case 'h':
            usage();
            return 0;
        case 't':
            autotest_id = atoi(optarg);
            mode = RUN_SINGLE_TEST;
            break;
        case 'p':
            package_id = atoi(optarg);
            mode = RUN_SINGLE_PACKAGE;
            break;
        case 'r':
            mode = RUN_ALL_RANDOM;
            break;
        case 'L':
            // list packages, scripts and exit
            print_test_list();
            return 0;
        case 'l':
            // list only packages and exit
            print_package_list();
            return 0;
        case 'x':
            stop_on_fail = 1;
            break;
        case 's':
            mode = RUN_SEARCH;
            strncpy(search_string, optarg, 128);
            search_string[127] = '\0';
            break;
        case 'v':
            verbose = 1;
            liquid_autotest_verbose = 1;
            break;
        case 'q':
            verbose = 0;
            liquid_autotest_verbose = 0;
            break;
        case 'o':
            strncpy(filename,optarg,255);
            filename[255] = '\0';
            break;
        default:
            return 1;
        }
    }

    // validate results
    if (autotest_id >= NUM_AUTOSCRIPTS) {
        printf("error, cannot run autotest %u; index exceeded\n", autotest_id);
        return -1;
    } else if (package_id >= NUM_PACKAGES) {
        printf("error, cannot run package %u; index exceeded\n", package_id);
        return -1;
    }

    unsigned int n=0;
    switch (mode) {
    case RUN_ALL:
        for (i=0; i<NUM_PACKAGES; i++) {
            execute_package( &packages[i], verbose );

            n++;
            if (stop_on_fail && liquid_autotest_num_failed > 0)
                break;
        }

        for (i=0; i<n; i++) {
            if (verbose)
                print_package_results( &packages[i] );
        }
        break;
    case RUN_ALL_RANDOM:
        // initialize with large random number
        i = (rseed + 8191) % NUM_AUTOSCRIPTS;

        while (n < NUM_AUTOSCRIPTS) {
            i = (i + 524287) % NUM_AUTOSCRIPTS;
            while (scripts[i].executed) {
                i++;
                if (i >= NUM_AUTOSCRIPTS)
                    i %= NUM_AUTOSCRIPTS;
            }

            printf("executing test %4u (%4u / %4u)\n", i, n+1, NUM_AUTOSCRIPTS);
            execute_autotest( &scripts[i], verbose );

            n++;
            if (stop_on_fail && liquid_autotest_num_failed > 0)
                break;
        }

        for (i=0; i<n; i++) {
            if (verbose)
                print_autotest_results( &scripts[i] );
        }
        break;
    case RUN_SINGLE_TEST:
        execute_autotest( &scripts[autotest_id], verbose );
        if (verbose)
            print_autotest_results( &scripts[autotest_id] );
        break;
    case RUN_SINGLE_PACKAGE:
        execute_package( &packages[package_id], verbose );
        if (verbose)
            print_package_results( &packages[package_id] );
        break;
    case RUN_SEARCH:
        printf("running all scripts matching '%s'...\n", search_string);

        // search all packages
        for (i=0; i<NUM_PACKAGES; i++)
            execute_package_search( &packages[i], search_string, verbose);

        // print results
        for (i=0; i<NUM_PACKAGES; i++) {
            if (verbose && packages[i].executed)
                print_package_results( &packages[i] );
        }
        break;
    }

    if (liquid_autotest_verbose)
        print_unstable_tests();

    autotest_print_results();

    // export results
    if (strcmp(filename,"")!=0)
        export_results(filename);

    return 0;
}

// execute a specific autotest
//  _test       :   pointer to autotest object
//  _verbose    :   verbose output flag
void execute_autotest(autotest_t * _test,
                      int _verbose)
{
    unsigned long int autotest_num_passed_init = liquid_autotest_num_passed;
    unsigned long int autotest_num_failed_init = liquid_autotest_num_failed;
    unsigned long int autotest_num_warnings_init = liquid_autotest_num_warnings;

    // execute test
    _test->api();

    _test->num_passed = liquid_autotest_num_passed - autotest_num_passed_init;
    _test->num_failed = liquid_autotest_num_failed - autotest_num_failed_init;
    _test->num_warnings = liquid_autotest_num_warnings - autotest_num_warnings_init;
    _test->num_checks = _test->num_passed + _test->num_failed;
    _test->pass = (_test->num_failed==0) ? 1 : 0;
    if (_test->num_checks > 0)
        _test->percent_passed = 100.0f * (float) (_test->num_passed) / (float) (_test->num_checks);
    else
        _test->percent_passed = 0.0f;

    _test->executed = 1;

    //if (_verbose)
    //    print_autotest_results(_test);
}

// execute a specific package
//  _p          :   pointer to package object
//  _verbose    :   verbose output flag
void execute_package(package_t * _p,
                     int _verbose)
{
    if (_verbose)
        printf("%u: %s\n", _p->id, _p->name);
    
    unsigned int i;
    for (i=0; i<_p->num_scripts; i++) {
        execute_autotest( &scripts[ i + _p->index ], _verbose );
    }
    
    _p->executed = 1;
}

// execute a specific package if string matches
void execute_package_search(package_t * _p, char * _str, int _verbose)
{
    // see if search string matches autotest name
    if (strstr(_p->name, _str) != NULL) {
        // run the package
        execute_package(_p, _verbose);
    } else {

        unsigned int i;
        unsigned int i0 = _p->index;
        unsigned int i1 = _p->num_scripts + i0;
        for (i=i0; i<i1; i++) {
            // see if search string matches autotest name
            if (strstr(scripts[i].name, _str) != NULL) {
                // run the autotest
                execute_autotest( &scripts[i], _verbose );
                _p->executed = 1;
            }
        }
    }
}

// print results of a particular test
void print_autotest_results(autotest_t * _test)
{
    if (!_test->executed)
        printf("    %3u :   IGNORED ", _test->id);
    else if (_test->pass)
        printf("    %3u :   PASS    ", _test->id);
    else
        printf("    %3u : <<FAIL>>  ", _test->id);

    printf("passed %4lu / %4lu checks (%5.1f%%) : %s\n",
            _test->num_passed,
            _test->num_checks,
            _test->percent_passed,
            _test->name);
}

// print results of a particular package
void print_package_results(package_t * _p)
{
    unsigned int i;
    printf("%u: %s:\n", _p->id, _p->name);
    for (i=_p->index; i<(_p->index+_p->num_scripts); i++) {
        //if ( scripts[i].executed ) // only print scripts that were executed
        print_autotest_results( &scripts[i] );
    }

    printf("\n");
}

// print all unstable tests (those which failed or gave warnings)
void print_unstable_tests(void)
{
    if (liquid_autotest_num_failed == 0 &&
        liquid_autotest_num_warnings == 0)
    {
        return;
    }

    printf("==================================\n");
    printf(" UNSTABLE TESTS:\n");
    unsigned int t;
    for (t=0; t<NUM_AUTOSCRIPTS; t++) {
        if (scripts[t].executed) {
            if (!scripts[t].pass) {
                printf("    %3u : <<FAIL>> %s\n", scripts[t].id,
                                                  scripts[t].name);
            }
            
            if (scripts[t].num_warnings > 0) {
                printf("    %3u : %4lu warnings %s\n", scripts[t].id,
                                                       scripts[t].num_warnings,
                                                       scripts[t].name);
            }
        }
    }
}

void print_test_list(void)
{
    unsigned int i;
    unsigned int j;
    for (i=0; i<NUM_PACKAGES; i++) {
        printf("%u: %s\n", packages[i].id, packages[i].name);
        for (j=packages[i].index; j<packages[i].num_scripts+packages[i].index; j++)
            printf("    %u: %s\n", scripts[j].id, scripts[j].name);
    }
}

void print_package_list(void)
{
    unsigned int i;
    for (i=0; i<NUM_PACKAGES; i++)
        printf("%u: %s\n", packages[i].id, packages[i].name);
}

// print basic autotest results to stdout
int export_results(char * _filename)
{
    // try to open file for writing
    FILE * fid = fopen(_filename,"w");
    if (!fid) {
        fprintf(stderr,"error: export_results(), could not open '%s' for writing\n", _filename);
        return -1;
    }

    // print header
    fprintf(fid,"{\n");
    fprintf(fid,"  \"build-info\" : {\n");
    fprintf(fid,"  },\n");
    fprintf(fid,"  \"tests\" : [\n");

    // print each individual test as opposed to package
    unsigned int i;
    for (i=0; i<NUM_AUTOSCRIPTS; i++) {
        fprintf(fid,"    {\"id\":%3u, \"pass\":%s, \"num_checks\":%4lu, \"num_passed\":%4lu, \"name\":\"%s\"}%s\n",
                scripts[i].id,
                scripts[i].num_failed == 0 ? "true" : "false",
                scripts[i].num_checks,
                scripts[i].num_passed,
                scripts[i].name,
                i==NUM_AUTOSCRIPTS-1 ? "" : ",");
    }

    fprintf(fid,"  ]\n");
    fprintf(fid,"}\n");
    fclose(fid);

    if (liquid_autotest_verbose)
        printf("output .json results written to %s\n", _filename);

    // everything is fine
    return 0;
}

