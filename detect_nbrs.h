/**
 * @author: Attilio priolo priolo[at]dia.uniroma3.it
 * @author: Ryan K. Williams rkwillia[at]usc.edu 
 * 
 * Detects graph neighbors with constraint-aware discernment zone
 */

#ifndef _DETECT_NBRS_H_
#define _DETECT_NBRS_H_

#include <math.h>
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include "input_management.h"
#include "globals_ryan.h"

/**
 * Init the detect neighborhood
 */
void init_detect_nbrs();

/**
 *Function to run the detect_nbrs algorithm
 */
void run_detect_nbrs();

//void print_int_squared_matrix(int **m,int n);
//void print_gsl_matrix(gsl_matrix *m);

/**
 *Free the memory up
 */
void close_detect_nbrs();


#endif