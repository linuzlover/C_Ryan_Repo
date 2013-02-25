#include "detect_nbrs.h"

int **adj_copy;

void init_detect_nbrs()
{
    int i;
    
    adj_copy=calloc(N,sizeof(int*));
    for(i=0; i<N; i++)
    {
        adj_copy[i]=calloc(N,sizeof(int));
    }
}


// //TODO: TO delete
// void print_gsl_matrix(gsl_matrix *m)
// {
//   int i,j;
//   
//   printf("Printing matrix\n [");
//   
//   for(i=0;i<m->size1;i++)
//   {
//     
//     for(j=0;j<m->size2;j++)
//     {
//       printf("%f,",gsl_matrix_get(m,i,j));
//     }
//     printf("\n");
//   }
// }
// 
// void print_int_squared_matrix(int **m,int n)
// {
//   int i,j;
//   
//   printf("Printing matrix\n [");
//   
//   for(i=0;i<n;i++)
//   {
//     
//     for(j=0;j<n;j++)
//     {
//       printf("%d,",m[i][j]);
//     }
//     printf("\n");
//   }
// }

void run_detect_nbrs()
{

    gsl_vector_view rij;
    double nrij;

    int i,j; //Indexes
    int ainbrs=0,ajnbrs=0; //number of agent i neighbors
    int aicands=0,ajcands=0,ai_del_cands=0,aj_del_cands=0; //number of agent i candidates
    //
    unsigned int violation=0;

    copy_squared_array_of_arrays(adj_copy,funcs_input.adj_mat,N);
    
    set_adj_to_zero();
    int k;
    
    for(i=0; i<N; i++)
    {
        for(j=0; j<N; j++)
        {
            if(i!=j)
            {
                rij=gsl_matrix_column(funcs_input.dists[i],j);
                
                nrij=gsl_blas_dnrm2((const gsl_vector *)&rij);
                
		
                // Is the jth agent within the interaction boundary
                if(nrij<=d2)
                {
                    // If j is already a neighbor, i.e. it has entered the
                    // connection zone prior, it remains a neighbor.  Or, if j has
                    // now entered the connection zone it becomes a neighbor.
                    if(adj_copy[i][j]>0 || nrij<=d1)
                        set_i_j_adjacency(1,i,j);
                }

                // Added a neighbor
                if(adj_copy[i][j]==0 && funcs_input.adj_mat[i][j]>0)
                    set_i_j_neighbors(1,i,j);
                //Lost a neighbor
                if(adj_copy[i][j]>0 && funcs_input.adj_mat[i][j]==0)
                    set_i_j_neighbors(0,i,j);
                //Contact has been lost or agent is a neighbor, clear
                // link creation constraint variables
                if(nrij>d2 || funcs_input.adj_mat[i][j]>0)
                {
                    set_i_j_to_stitch(0,i,j);
                    set_i_j_to_repel(0,i,j);
                }
                // Contact has been lost or agent within connection region, clear
                // link deletion constraint variables
                if(nrij>d2 || nrij<=d1)
                {
                    set_i_j_del_cand(0,i,j);
                    set_i_j_attract(0,i,j);

                }

            }
        }
    }


//          printf("To repel");
//      print_int_squared_matrix(funcs_input.to_repel,n);
    
    for(i=0; i<N; i++)
    {
        //ith agent neighbors
        ainbrs=get_row_sum(funcs_input.neighbors,i,N);
	//printf("ainbrs[%d]: %d\n",i,ainbrs);
	
        for(j=i+1; j<N; j++)
        {
            //jth agent neighbors
            ajnbrs=get_row_sum(funcs_input.neighbors,j,N);
	   // printf("ajnbrs[%d]: %d\n",j,ajnbrs);
            rij=gsl_matrix_column(funcs_input.dists[i],j);
            nrij=gsl_blas_dnrm2((const gsl_vector *)&rij);

            // Within the discernment zone and not an established neighbor
            if(nrij<= d2 && funcs_input.adj_mat[i][j]==0)
            {
                // Not a previous candidate nor a previous repelled agent
                if(funcs_input.to_stitch[i][j]==0 && funcs_input.to_repel[i][j]==0)
                {
                    aicands=get_row_sum(funcs_input.to_stitch,i,N);
		    //printf("aicands[%d]: %d\n",i,aicands);
	
                    ajcands=get_row_sum(funcs_input.to_stitch,j,N);
		    //printf("ajcands[%d]: %d\n",j,ajcands);
	
		    
                    violation=((ainbrs+aicands)>= M || (ajnbrs+ajcands) >= M);
// 		    printf("violation [%d]=%d\n",i,violation);
		    
                    if(violation)
                    {
		        set_i_j_to_repel(1,i,j);
                        set_i_j_to_repel(1,j,i);
                    }
                    else
                    {
                        set_i_j_to_stitch(1,i,j);
                        set_i_j_to_stitch(1,j,i);
                    }

                }
            }

            //Within the discernment region and an established neighbor
            if(nrij>d1 && funcs_input.adj_mat[i][j]>0)
            {
                //Not a previous candidate nor a previous attracted agent
                if(funcs_input.del_cand[i][j]==0 && 
funcs_input.attract[i][j]==0)
                {
                    //Compute number of candidates for each agent
                    ai_del_cands=get_row_sum(funcs_input.del_cand,i,N);
                    aj_del_cands=get_row_sum(funcs_input.del_cand,j,N);

                    //Check for min degree violation
                    violation=((ainbrs-ai_del_cands)<=W || (ajnbrs-aj_del_cands)<=W);

                    // Determine candidacy/attraction based on constraint status
                    if(violation)
                    {
                        set_i_j_attract(1,i,j);
                        set_i_j_attract(1,j,i);
                    }
                    else
                    {
                        set_i_j_del_cand(1,i,j);
                        set_i_j_del_cand(1,j,i);
                    }

                }
            }
        }
    }
    
//      printf("To repel");
//      print_int_squared_matrix(funcs_input.to_repel,n);
// 
//     printf("Neighbors");
//     print_int_squared_matrix(funcs_input.neighbors,n);
// 
//     printf("Del Cand");
//     print_int_squared_matrix(funcs_input.del_cand,n);
// 
//     printf("Cands");
//     print_int_squared_matrix(funcs_input.to_stitch,n);
// 
//     printf("Attract");
//     print_int_squared_matrix(funcs_input.attract,n);


}

void close_detect_nbrs()
{
    int i;
    
    for(i=0; i<N; i++)
    {
        free(adj_copy[i]);
    }
    free(adj_copy);
}
