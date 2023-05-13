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

/* Some convenient definitions. */
#define NX          ACADO_NX  /* 13 Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

#define NUM_STEPS   10        /* Number of real-time iterations. */
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


    //*******************************************************************************//
    //****************************** txt File Read Start ****************************//
    //*******************************************************************************//
    
     //****************************** mesure.txt reading ******************************//
    // FILE object - https://www.tutorialspoint.com/cprogramming/c_file_io.htm
    FILE *mesure; // Read measurements
    mesure = fopen("mesure.txt", "r"); // "r" means for reading
    if (mesure == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }

    int rows = 1; 
    int cols = 13;
    float numberArray_x0[rows*cols]; // 13 columns and 1 rows to read
    int ii,j;

    for (ii = 0; ii < rows*cols; ii++){
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(mesure, "%e", &numberArray_x0[ii]);
        //printf("%e  ", numberArray_x0[ii]);
    
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

    //****************************** ref.txt reading ******************************/

    FILE *ref;
    ref = fopen("ref.txt", "r");
    if (ref == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    } 
    
    rows = N+1; 
    cols = 16;  
    float ref_numberArray_y[rows*cols]; // 16 columns and 20 rows to read
    
    for (ii = 0; ii < rows*cols; ii++){ 
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(ref, "%e", &ref_numberArray_y[ii]);
        //printf("%e  ", ref_numberArray_y[ii]);
       
    }
    
    fclose(ref);
    rows = 1; 
    cols = 14;  
    float ref_numberArray_yN[rows*cols]; // 14 columns and 1 rows to read
    
    for (ii = (((N+1)*16)-16); ii < (((N+1)*16)-4); ii++){
    
    ref_numberArray_yN[ii] = ref_numberArray_y[ii];
    //printf("%e  ", ref_numberArray_yN[ii]);


    }


    //*******************************************************************************//
    //****************************** txt File Read End ******************************//
    //*******************************************************************************//

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = ref_numberArray_y[i];
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[ i ] = ref_numberArray_yN[i];

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < NX; ++i) acadoVariables.x0[ i ] = numberArray_x0[i];
#endif

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
