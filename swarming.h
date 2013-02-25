/**
 * @author: Attilio priolo priolo[at]dia.uniroma3.it
 * @author: Ryan K. Williams rkwillia[at]usc.edu 
 * 
 * This Header file contains the definition related with the swarming functions.
 * Commenting out the main function in the .c source code, the header file along with the implementation one can be used to generate a library.
 * A detailed description will be added shortly
 */
#ifndef _SWARMING_H_
#define _SWARMING_H_

#include <math.h>
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include "input_management.h"
#include "globals_ryan.h"


float ca_coeffs[2]; /**< Collision avoidance coefficients. */ 

float lm_coeffs[2]; /**< Link maintenance coefficients. */ 

float cs_coeffs[2]; /**< Constraint satisfaction coefficients. */ 

float cp_coeffs[2]; /**< Candidate potential coefficients. */ 



 /** Function to init the coefficients' arrays.
 * This function should be called only once. The number of robots is assumed to be constant.
  * @return none
 */ 
void init_swarming();

/**
 *Main function of the swarming algorithm. This function assumes that both positions and velocities_res are allocated in memory. 
 *The function is not in charge of deallocating such data structures.
 *@param[out] velocities_res A gsl_matrix containing the resulting speeds. It has the same dimensions as positions.
 */
void run_swarming(gsl_matrix *velocities_res);

/**
 * Function to clear the memory after the use of this algorithm.
 * This function should be called only once.
 */ 
void close_swarming();

#endif
