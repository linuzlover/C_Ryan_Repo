/**
 * @author: Attilio priolo priolo[at]dia.uniroma3.it
 * @author: Ryan K. Williams rkwillia[at]usc.edu 
 * 
 * This Header file contains the structures used as inputs for the other functions
 * A detailed description will be added shortly
 */
#ifndef _INPUT_H_
#define _INPUT_H_

#include <math.h>
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include "globals_ryan.h"



int N; /**<Number of agents */


/**
 * A struct representing the input to the swarming algorithm.
 * */
typedef struct{
 
  gsl_matrix *positions; /**<A dim x n  matrix with the positions of the agents*/
  gsl_matrix **dists; /**<A dim x n  matrix with the positions of the agents*/
  int **adj_mat;  /**<Array of arrays representing the adjacency relationship (it has to be a symmetric matrix)*/
  int **to_repel; /**<Array of arrays representing the repulsive relationship (it has to be a symmetric matrix)*/
  int **to_stitch;/**<Array of arrays representing the stitching relationship (it has to be a symmetric matrix)*/
  int **neighbors;/**<Array of arrays representing the neighbors (it has to be a symmetric matrix)*/
  int **del_cand;/**<Array of arrays representing the candidates for deletion relationship (it has to be a symmetric matrix)*/
  int **attract;/**<Array of arrays representing the candidates to be attracted(it has to be a symmetric matrix)*/
} input_functions;



/**
 *Actual structure
 */
input_functions funcs_input;

/**
 * This function is intended to init the input management system. It must be called only once.
 */
void init_input_management(int n);

/**
 * Set the i,j element of the adjacency matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_adjacency(int value, int i, int j);

/**
 * Set the i,j element of the repulsion matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_to_repel(int value, int i, int j);

/**
 * Set the i,j element of the neighbors matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_neighbors(int value, int i, int j);

/**
 * Set the i,j element of the stitching matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_to_stitch(int value, int i, int j);

/**
 * Set the i,j element of the attract matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_attract(int value, int i, int j);

/**
 * Set the i,j element of the delete candidate matrix
 *@param[in] value Value to set
 *@param[in] i row
 *@param[in] j column
 */
void set_i_j_del_cand(int value, int i, int j);

/**
 * Reset the adjacency matrix
 */
void set_adj_to_zero();

/**
 * Copy a squared matrix of dimension "dimension"
 *@param[out] dest Matrix to copy into
 *@param[in] src Matrix to be copied
 *@param[in] dimension Dimension of the matrix
 */
void copy_squared_array_of_arrays(int **dest,int **src,int dimension);

/**
 * Compute the sum on the ith column of a square matrix
 *@param[out] data Src matrix
 *@param[in] i ith column
 *@param[in] size Size of the matrix
 */
int get_column_sum(int **data,int i,int size);
/**
 * Compute the sum on the ith row of a square matrix
 *@param[out] data Src matrix
 *@param[in] i ith row
 *@param[in] size Size of the matrix
 */
int get_row_sum(int **data,int i,int size);

/**
 *Function to be called after setting all the robots positions to compute the inter-distances.
 */
void input_management_update_dists();

/**This function is intended to free the memory of this library
 *
 */
void close_input_management();

#endif
