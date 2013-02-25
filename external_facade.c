#include "external_facade.h"

void print_matr(int **matrix,int number)

{
	int i,j;
	
	printf("\n[ ");
	
	for(i=0;i<number;i++)
	{
		for(j=0;j<number;j++)	
		{
			printf("%d,\t",matrix[i][j]);			
		}
		printf("\n");
	}
	printf(" ]\n");
}

void fprint_matr(FILE *log,int **matrix,int number)
{
	int i,j;
	
	fprintf(log,"\n[ ");
	
	for(i=0;i<number;i++)
	{
		for(j=0;j<number;j++)	
		{
			fprintf(log,"%d,\t",matrix[i][j]);			
		}
		fprintf(log,"\n");
	}
	fprintf(log," ]\n");
}

void ryan_init(int n)
{
    init_input_management(n);
    init_detect_nbrs();
    init_swarming();
}

void set_i_column_position(double* position, int i)
{
    int j;
    for(j=0; j<dim; j++)
    {
        gsl_matrix_set(funcs_input.positions,j,i,position[j]);
    }
}

void run_algorithm(gsl_matrix *velocities_res)
{
  input_management_update_dists();
  run_detect_nbrs();
  run_swarming(velocities_res);
}

void ryan_close()
{
    close_detect_nbrs();
    close_input_management();
    close_swarming();
}
