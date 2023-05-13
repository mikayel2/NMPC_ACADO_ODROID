/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */



/*

IMPORTANT: This file should serve as a starting point to develop the user
code for the OCP solver. The code below is for illustration purposes. Most
likely you will not get good results if you execute this code without any
modification(s).

Please read the examples in order to understand how to write user code how
to run the OCP solver. You can find more info on the website:
www.acadotoolkit.org

*/

#include "acado_common.h"
#include "acado_auxiliary_functions.h"

#include <stdio.h>
#include <stdlib.h>

// Type test function
#define PRINT_VAR_TYPE(x) printf("Type of " #x " is %s\n", \
    _Generic((x), \
        int: "int", \
        float: "float", \
        double: "double", \
        char: "char", \
        default: "unknown" \
    ))

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   1        /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

/* A template for testing of the solver. */
int main( )
{
	/* Some temporary variables. */
	int    i, iter;
	acado_timer t;

	/* Initialize the solver. */
	acado_initializeSolver();

    // FILE object - https://www.tutorialspoint.com/cprogramming/c_file_io.htm
    FILE *mesure; // Read measurements
    mesure = fopen("mesure.txt", "r"); // "r" means for reading
    
    // Check whether file is empty or not
    if (mesure == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }
    int rows = 30+2500; 
    int cols = 14;
    float numberArray[rows][cols]; // 18 columns and 17 rows to read
    int ii,j;
    
    
    for (ii = 0; ii < rows; ii++){ // rows
    for (j = 0; j < cols; j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(mesure, "%e", &numberArray[ii][j]);
    }
    }
    // Close the file
    fclose(mesure);
    // Print for test
    //for (ii = 0; ii < rows; ii++){ // rows
    //for (j = 0; j < cols; j++)  // columns
    //{
    //    printf("%e  ", numberArray[ii][j]);
        
    //}
    //    printf(" \n", numberArray[ii][j]);
        
    //}
    
    FILE *ref;
    ref = fopen("ref.txt", "r"); // "r" means for reading
    
    // Check whether file is empty or not
    if (ref == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }
    rows = (N+1)+2500; 
    cols = 18;  
    float ref_numberArray[rows][cols]; // 18 columns and 17 rows to read
    int cols_x = NX * (N + 1);
    float ref_numberArray_x[cols_x];
    //cols = NU * N;
    //float ref_numberArray_u[cols];
    //cols = NY * N;
    //float ref_numberArray_y[cols];
    //cols = NYN;
    //float ref_numberArray_yN[cols];
    //int ii,j;
    
    // Fill the whole N+1 sample points from ref.txt as 2D array
    for (ii = 0; ii < rows; ii++){ // rows
    for (j = 0; j < cols; j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(ref, "%e", &ref_numberArray[ii][j]);
    
       
    }
    }
    // Fill N+1 sample points from ref.txt as 1D array to fill the acadoVariables.x 
    int sv = 0;
    for (ii = 0; ii < (N+1); ii++){ // rows
    for (j = 1; j < (NX+1); j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        ref_numberArray_x[sv] = ref_numberArray[ii][j];
        //printf("%e  ", ref_numberArray_x[sv]);
        sv = sv + 1;
        
  
    }
    //printf(" \n");
    }

    // Fill N sample points from ref.txt as 1D array to fill the acadoVariables.u
    int cols_u = NU * N;
    float ref_numberArray_u[cols_u];
    sv = 0;
    for (ii = 0; ii < (N); ii++){ // rows
    for (j = 14; j < (NU+14); j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        ref_numberArray_u[sv] = ref_numberArray[ii][j];
        //printf("%e  ", ref_numberArray_u[sv]);
        sv = sv + 1;
        
  
    }
    //printf(" \n");
    }

    int cols_y = NY * N;
    float ref_numberArray_y[cols_y];
    sv = 0;
    for (ii = 0+30+2500; ii < (N)+30+2500; ii++){ // rows
    for (j = 1; j < (NY+1); j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        ref_numberArray_y[sv] = ref_numberArray[ii][j];
        //printf("%e  ", ref_numberArray_y[sv]);
        sv = sv + 1;
        
  
    }
    //printf(" \n");
    }

    int cols_yN = NYN;
    float ref_numberArray_yN[cols_yN];
    sv = 0;
    for (ii = 0+30+2500; ii < 1+30+2500; ii++){ // rows
    for (j = 1; j < (NYN+1); j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        ref_numberArray_yN[sv] = ref_numberArray[ii][j];
        //printf("%e  ", ref_numberArray_yN[sv]);
        sv = sv + 1;
        
  
    }
    //printf(" \n");
    }

    // Print for test
    //for (ii = 0; ii < rows; ii++){ // rows
    //for (j = 0; j < cols; j++)  // columns
    //{
        //printf("%e  ", ref_numberArray[ii][j]);
        
    //}
        //printf(" \n", ref_numberArray[ii][j]);
        
    //}

	/* Initialize the states and controls. */
    //for (ii=0; ii < (N+1); ++i){
    //for (i = 0; i < NX ; ++i){


    //}
    //}

    //size_t size = sizeof(ref_numberArray_x);
    //printf("The size is %d \n", size);
    




	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;//ref_numberArray_x[i];
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0; // ref_numberArray_u[i];

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = ref_numberArray_y[i];
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = ref_numberArray_yN[i];

	/* MPC: initialize the current state feedback. */
//#if ACADO_INITIAL_STATE_FIXED   
	for (i = 0; i < NX; ++i) {
    
    acadoVariables.x0[ i ] = numberArray[2500][i+1]; // 1 row , 13 columns (NX = 13)
                                                // x, x_dot, q, omega_B
    printf("%e  ", numberArray[2500][i+1]);
    
    }
    
//#endif

	if( VERBOSE ) acado_printHeader();

	/* Prepare first step */
	acado_preparationStep();

	/* Get the time before start of the loop. */
	acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

		/* Optional: shift the initialization (look at acado_common.h). */
        /* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */

		/* Prepare for the next step. */
		acado_preparationStep();
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);

	acado_printDifferentialVariables();
	acado_printControlVariables();

    return 0;
}
