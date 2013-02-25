#include "swarming.h"

//File Scope variable hiding!

/*Control gains*/
static const float k[] = {1,0.5,5.0,1.0,0.01}; /**< Swarming, Link maintenance, Collision avoidance, Discriminative link creation, Stitch control*/

/*Regions' radii*/




gsl_vector *rij;//positions difference vector
gsl_vector *rij_temp;//positions difference vector (temporary copy to avoid overwritings)
//static const int input_num=5; /**<Number of inputs*/
gsl_vector *u[input_num]; //Vector containing the different inputs

void print_gsl_vector(gsl_vector *toPrint)
{
    int length,i;
    double curr;
    length=toPrint->size;

    for(i=0; i<length; i++)
    {
        curr=gsl_vector_get(toPrint,i);
        printf("vec[%d]: %f\n",i,curr);
    }
}

/**
* Coefficients initialization function. The coefficients are chosen to guarantee the smoothness of the potentials.
* @see Matlab source code
*/
void init_coefficients()
{

    //Collision avoidance potential coefficients
    ca_coeffs[0]=-3/powf(d0,4);
    ca_coeffs[1]=8/powf(d0,4);

    //Link maintenance potential coefficients
    lm_coeffs[0]=(3*d1*d1 + d2*d2)/powf((d1*d1 - d2*d2),3);
    lm_coeffs[1]=-(8*powf(d1,3))/powf(d1*d1 - d2*d2,3);


    //Constraint satisfaction potential coefficients
    cs_coeffs[0]=(d1*d1 + 3*d2*d2)/powf(d1*d1 - d2*d2,3);
    cs_coeffs[1]=(8*powf(d2,3))/powf(-d1*d1 + d2*d2,3);


    //Candidate potential coefficients
    cp_coeffs[0]=(k[0]*d1)/(d1 - d2);
    cp_coeffs[1]=-((2*k[0]*d1*d2)/(d1 - d2));

}

void init_data_structures()
{
    int i;
    rij=gsl_vector_alloc(dim);
    rij_temp=gsl_vector_alloc(dim);

    /*Input vectors initialization*/
    for(i=0; i<input_num; i++)
    {
        u[i]=gsl_vector_alloc(dim);
    }

}

/**
* This function set all the elements of the array of the gsl_vector to zero.
*/
void zero_inputs(gsl_vector **u,int input_n)
{
    int i;

    for(i=0; i<input_n; i++)
    {
        gsl_vector_set_zero(u[i]);
    }
}


void run_swarming(gsl_matrix *velocities_res)
{
    //Initializations
    int i,j; //indexes
    int aij; //adjacency matrix element
    double nrij=0.0; //norm of the difference vector
    gsl_vector_view column_i;

    double temp_coeffs[input_num]; //Temporary double variable used in the computation of u2

    //Iterating on all the agents
    for(i=0; i<N; i++)
    {

        zero_inputs(u,input_num);

        //Iterating on the neighbors
        for(j=0; j<N; j++)
        {
            //Each robot is not considering himself
            if(i!=j)
            {

	      
		column_i=gsl_matrix_column(funcs_input.dists[i],j);
		gsl_vector_memcpy(rij,(const gsl_vector *)&column_i);
		
	        //Computing the norm of the difference vector
                nrij=gsl_blas_dnrm2((const gsl_vector *)rij);
                //Extracting the i,j element of the Adjancency matrix
                aij=funcs_input.adj_mat[i][j];
                if(aij>0)
                {
                    //U1
                    gsl_vector_memcpy(rij_temp,rij);
                    gsl_vector_scale(rij_temp,2.0);
                    gsl_vector_add(u[0],rij_temp);

                    //U2
                    if(nrij>=d1) {
                        temp_coeffs[1]=(2.0/powf(nrij*nrij-d2*d2,2.0) + 2*lm_coeffs[0]+ lm_coeffs[1]/nrij);
                        gsl_vector_memcpy(rij_temp,rij);
                        gsl_vector_scale(rij_temp,temp_coeffs[1]);
                        gsl_vector_add(u[1],rij_temp);
                    }
                    //U3
                    if(nrij <= d0) {
                        temp_coeffs[2]=(-2.0/powf(nrij,4.0)+2*ca_coeffs[0]+ca_coeffs[1]/nrij);
                        gsl_vector_memcpy(rij_temp,rij);
                        gsl_vector_scale(rij_temp,temp_coeffs[2]);
                        gsl_vector_add(u[2],rij_temp);
                    }

                }

                //U4
                if(funcs_input.to_repel[i][j]==1)
                {
                    temp_coeffs[3]=(-2.0/powf(nrij*nrij-d1*d1,2.0)+2*cs_coeffs[0]+cs_coeffs[1]/nrij);
                    gsl_vector_memcpy(rij_temp,rij);
                    gsl_vector_scale(rij_temp,temp_coeffs[3]);
                    gsl_vector_add(u[3],rij_temp);
                }

                //U5
                if(funcs_input.to_stitch[i][j]==1)
                {
                    temp_coeffs[4]=(2.0*cp_coeffs[0]+cp_coeffs[1]/nrij);
                    gsl_vector_memcpy(rij_temp,rij);
                    gsl_vector_scale(rij_temp,temp_coeffs[4]);
                    gsl_vector_add(u[4],rij_temp);
                }

            }

        }

        //Multiply each input for its constant k
        for(j=0; j<input_num; j++)
        {
            gsl_vector_scale(u[j],-k[j]);
        }

        //Summing up all the inputs
        for(j=1; j<input_num; j++)
        {
            gsl_vector_add(u[0],u[j]);
        }

       // print_gsl_vector(u[0]);
        //Setting the corresponding column in the resulting velocities_res matrix
        gsl_matrix_set_col(velocities_res,i,u[0]);

    }


}

void init_swarming()
{
    
    //Initialization of the data structures
    init_data_structures();
    //Init the coefficients
    init_coefficients();
}


void close_swarming()
{
    int i;
    gsl_vector_free(rij);
    gsl_vector_free(rij_temp);
    for(i=0; i<input_num; i++)
    {
        gsl_vector_free(u[i]);
    }
}



//---------------------------
