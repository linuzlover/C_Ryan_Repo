#include "input_management.h"


void init_input_management(int n)
{
  int i;
  
  N=n;
  
  funcs_input.positions=gsl_matrix_alloc(dim,N);
  funcs_input.neighbors=calloc(N,sizeof(int*));
  funcs_input.adj_mat=calloc(N,sizeof(int*));
  funcs_input.to_repel=calloc(N,sizeof(int*));
  funcs_input.to_stitch=calloc(N,sizeof(int*));
  funcs_input.del_cand=calloc(N,sizeof(int*));
  funcs_input.attract=calloc(N,sizeof(int*));
  funcs_input.dists=calloc(N,sizeof(gsl_matrix*));
  
   for(i=0; i<N; i++)
    {
	funcs_input.neighbors[i]=calloc(N,sizeof(int));
        funcs_input.adj_mat[i]=calloc(N,sizeof(int));
        funcs_input.to_repel[i]=calloc(N,sizeof(int));
        funcs_input.to_stitch[i]=calloc(N,sizeof(int));
	funcs_input.attract[i]=calloc(N,sizeof(int));
        funcs_input.del_cand[i]=calloc(N,sizeof(int));
	funcs_input.dists[i]=gsl_matrix_alloc(dim,N);
    }
    
}

void close_input_management()
{
  int i;
  gsl_matrix_free(funcs_input.positions);
  
  for(i=0;i<N;i++)
  {
    free(funcs_input.neighbors[i]);
    free(funcs_input.adj_mat[i]);
    free(funcs_input.to_repel[i]);
    free(funcs_input.to_stitch[i]);
    free(funcs_input.del_cand[i]);
    free(funcs_input.attract[i]);
    gsl_matrix_free(funcs_input.dists[i]);
  }
  
  free(funcs_input.neighbors);
  free(funcs_input.adj_mat);
  free(funcs_input.to_repel);
  free(funcs_input.to_stitch);
  free(funcs_input.del_cand);
  free(funcs_input.attract);
  free(funcs_input.dists);
  
  
  
}

void input_management_update_dists()
{
  int i,j;
  
  gsl_vector_view column_i; //column_i of the positions matrix
  gsl_vector_view column_j; //column_j of the positions matrix
  gsl_vector *rij=gsl_vector_alloc(dim); //column_j of the positions matrix
  
  for(i=0;i<N;i++)
  {
    //Extracting the i-th column of the positions matrix
    column_i=gsl_matrix_column(funcs_input.positions,i);
    for(j=0;j<N;j++)
    {
      if(i!=j)
      {
	column_j=gsl_matrix_column(funcs_input.positions,j);
	gsl_vector_memcpy(rij,(const gsl_vector *)&column_i);
	gsl_vector_sub(rij,(const gsl_vector *)&column_j);
	gsl_matrix_set_col(funcs_input.dists[i],j,rij);
      }
    }
  }
  
  gsl_vector_free(rij);
}

void set_i_j_adjacency(int value, int i, int j)
{
    funcs_input.adj_mat[i][j]=value;
}

void set_i_j_to_repel(int value, int i, int j)
{
    funcs_input.to_repel[i][j]=value;
}

void set_i_j_to_stitch(int value, int i, int j)
{
    funcs_input.to_stitch[i][j]=value;
}

void set_i_j_neighbors(int value, int i, int j)
{
  funcs_input.neighbors[i][j]=value;
}

void set_i_j_attract(int value, int i, int j)
{
  funcs_input.attract[i][j]=value;
}

void set_i_j_del_cand(int value, int i, int j)
{
  funcs_input.del_cand[i][j]=value;
}


void set_adj_to_zero()
{
  int i,j;
  for(i=0;i<N;i++)
  {
    for(j=0;j<N;j++)
    {
      set_i_j_adjacency(0,i,j);
    }
  }
}


void copy_squared_array_of_arrays(int **dest,int **src,int dimension)
{
  int i,j;
  
  for(i=0;i<dimension;i++)
  {
    for(j=0;j<dimension;j++)
      dest[i][j]=src[i][j];
  }
}

int get_row_sum(int **data,int i,int size)
{
  int j;
  int acc=0;
  
  for(j=0;j<N;j++)
  {
    acc=acc+data[i][j];
  }
  
  return acc;
}

int get_column_sum(int **data,int i,int size)
{
  int j;
  int acc=0;
  
  for(j=0;j<N;j++)
  {
    acc=acc+data[j][i];
  }
  
  return acc;
}