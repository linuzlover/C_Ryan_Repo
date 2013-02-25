#ifndef _FACADE_H_
#define _FACADE_H_


#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <stdio.h>
#include "input_management.h"
#include "swarming.h"
#include "detect_nbrs.h"
#include "globals_ryan.h"

void print_matr(int **matrix,int number);

void fprint_matr(FILE *log,int **matrix,int number);


/**
 *This function initializes the data structure in order to run the detect_nbrs and dyn_swarming algorithms.
 * It must be called only once.
 */
void ryan_init(int n);


/**
 *Set the (ith) column of the position matrix.
 *@param[in] position vector of dim elements representing the ith robot position
 *@param[in] i index of the robot
 */
void set_i_column_position(double* position, int i);

/**
 *Main method. It updates the distance matrix, runs the detect_nbrs and runs the dyn_swarming algorithm.
 *@param[out] velocities_res A gsl matrix with the output speed (a "dim x n" matrix where dim is the state space dimension and n is the number of agents).
 */
void run_algorithm(gsl_matrix *velocities_res);

/**
 *Function to free the memory
 */
void ryan_close();

#endif
