/**
 * Main to perform tests. (without comments because they are not needed. this function is only temporary)
 */

#include <math.h>
#include <stdio.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include "external_facade.h"
#include "swarming.h"
#include "detect_nbrs.h"
#include "globals_ryan.h"

static const float dt=0.1;
static const int iters=3;
#define num_agents 5

//Init Positions
double *pos[num_agents];

void print_gsl_matrix(gsl_matrix *m)
{
  int i,j;
  
  printf("Printing matrix\n [");
  
  for(i=0;i<m->size1;i++)
  {
    
    for(j=0;j<m->size2;j++)
    {
      printf("%f,",gsl_matrix_get(m,i,j));
    }
    printf("\n");
  }
}

void move_agents(gsl_matrix *velocities_res)
{
  int i;
  gsl_vector_view column_i;
  
  for(i=0;i<num_agents;i++)
  {
    column_i=gsl_matrix_column(velocities_res,i);
    pos[i][0]=pos[i][0]+dt*gsl_vector_get((gsl_vector*)&column_i,0);
    pos[i][1]=pos[i][1]+dt*gsl_vector_get((gsl_vector*)&column_i,1);
  }
}

int main(int argc,char** argv)
{
  
  int i,j;
  
  
  //Allocating the matrix containing the swarming speeds
  gsl_matrix *velocities_res=gsl_matrix_alloc(dim,5);
  
  for(i=0;i<num_agents;i++)
  {
    pos[i]=calloc(dim,sizeof(double));
  }
  
  pos[0][0]= 3;
  pos[0][1]= 0;
  pos[1][0]= 5;
  pos[1][1]= 2;
  pos[2][0]= 3;
  pos[2][1]= 7;
  pos[3][0]= 1;
  pos[3][1]= 4;
  pos[4][0]= -1;
  pos[4][1]= 0;
  
  //Init the data structures
  ryan_init(num_agents);
  
  
  //Set the positions
  
  printf("%d ENNE\n",N);
  //Populating the data structures
  for(i=0; i<N; i++)
  {
    //Set the positions
    
    set_i_column_position(pos[i],i);
    for(j=0; j<N; j++)
    {
      if(j==(i+1)%N)
      {
	
	set_i_j_adjacency(0,i,j);
	set_i_j_to_repel(0,i,j);
	set_i_j_to_stitch(0,i,j);
	set_i_j_neighbors(0,i,j);
	set_i_j_attract(0,i,j);
	set_i_j_del_cand(0,i,j);
	
      }
      else
      {
	set_i_j_adjacency(0,i,j);
	set_i_j_to_repel(0,i,j);
	set_i_j_to_stitch(0,i,j);
	set_i_j_neighbors(0,i,j);
	set_i_j_attract(0,i,j);
	set_i_j_del_cand(0,i,j);
      }
    }
    
  }
  
  
  
  for(i=0;i<iters;i++)
  {
    run_algorithm(velocities_res);
    move_agents(velocities_res);
    print_gsl_matrix(velocities_res);
  }
  
  ryan_close();
  gsl_matrix_free(velocities_res);
  for(i=0;i<num_agents;i++)
  {
    free(pos[i]);
  }
  return 0;
}
