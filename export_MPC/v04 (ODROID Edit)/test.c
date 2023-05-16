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

#define NUM_STEPS   10       /* Number of real-time iterations. */
#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

// Float to int - round up
#define FLOAT_TO_INT(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

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
     

    //***********************************************************************//
    //********************* For Choosing ref. sample Start*******************//
    //***********************************************************************//  

    //float dt_ref = 0.005; // [s]
    //int jjj = 0;
    int up_smaple = 1; // from ref.
    //int low_smaple = 0; // from ref.
    float time_saver = 0; // [s] to track time passed during NMPC command calculation 

    //**********************************************************************//
    //********************* For Choosing ref. sample End*******************//
    //*********************************************************************//

    //*******************************************************************************//
    //****************************** txt File Read Start ****************************//
    //*******************************************************************************//
    
    // Variables to store mesure.txt file data
    int size_m = 3001;
    float xx_m[size_m];
    float xy_m[size_m];
    float xz_m[size_m];
    float vx_m[size_m];
    float vy_m[size_m];
    float vz_m[size_m];
    float q1_m[size_m];
    float q2_m[size_m];
    float q3_m[size_m];
    float q4_m[size_m];
    float wx_m[size_m];
    float wy_m[size_m];
    float wz_m[size_m];
    float numberArray_x0[NX]; // 13 columns and 1 rows to read

    FILE *mesure; // Read measurements
    mesure = fopen("mesure.txt", "r"); // Open to read mesure.txt file

    if (mesure == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }
    
    for (i = 0; i < size_m; i++){
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(mesure, "%e", &xx_m[i]);
        fscanf(mesure, "%e", &xy_m[i]);
        fscanf(mesure, "%e", &xz_m[i]);
        fscanf(mesure, "%e", &vx_m[i]);
        fscanf(mesure, "%e", &vy_m[i]);
        fscanf(mesure, "%e", &vz_m[i]);
        fscanf(mesure, "%e", &q1_m[i]);
        fscanf(mesure, "%e", &q2_m[i]);
        fscanf(mesure, "%e", &q3_m[i]);
        fscanf(mesure, "%e", &q4_m[i]);  
        fscanf(mesure, "%e", &wx_m[i]);
        fscanf(mesure, "%e", &wy_m[i]);
        fscanf(mesure, "%e", &wz_m[i]); 
   
        // Print to test
        //printf("%e  ", xx_m[i]);
        //printf("%e  ", xy_m[i]);
        //printf("%e  ", xz_m[i]);
        //printf("%e  ", vx_m[i]);
        //printf("%e  ", vy_m[i]);
        //printf("%e  ", vz_m[i]);
        //printf("%e  ", q1_m[i]);
        //printf("%e  ", q2_m[i]);
        //printf("%e  ", q3_m[i]);
        //printf("%e  ", q4_m[i]);
        //printf("%e  ", wx_m[i]);
        //  printf("%e  ", wy_m[i]);
        //printf("%e  ", wz_m[i]);
     }

     fclose(mesure); // Close mesure.txt file

    // Print test 
    //int row_m = 10;
    //printf("\t mesure.txt file row %d: Value for xx = %.3e\n\n", row_m, xx_m[row_m]);
    //printf("\t mesure.txt file row %d: Value for xy = %.3e\n\n", row_m, xy_m[row_m]);
    //printf("\t mesure.txt file row %d: Value for xz = %.3e\n\n", row_m, xz_m[row_m]);


    // Variables to store mesure.txt file data
    int size_r = 3001;
    float xx_r[size_r];
    float xy_r[size_r];
    float xz_r[size_r];
    float vx_r[size_r];
    float vy_r[size_r];
    float vz_r[size_r];
    float q2_r[size_r];
    float q3_r[size_r];
    float q4_r[size_r];
    float wx_r[size_r];
    float wy_r[size_r];
    float wz_r[size_r];
    float u1_r[size_r];
    float u2_r[size_r];
    float u3_r[size_r];
    float u4_r[size_r];
    float ref_numberArray_y[NY*N]; // 16 columns and 20 rows to read
    float ref_numberArray_yN[NYN]; // 14 columns and 1 rows to read


    FILE *ref;
    ref = fopen("ref.txt", "r");
    if (ref == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    } 

    for (i = 0; i < size_m; i++){
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(ref, "%e", &xx_r[i]);
        fscanf(ref, "%e", &xy_r[i]);
        fscanf(ref, "%e", &xz_r[i]);
        fscanf(ref, "%e", &vx_r[i]);
        fscanf(ref, "%e", &vy_r[i]);
        fscanf(ref, "%e", &vz_r[i]);
        fscanf(ref, "%e", &q2_r[i]);
        fscanf(ref, "%e", &q3_r[i]);
        fscanf(ref, "%e", &q4_r[i]);  
        fscanf(ref, "%e", &wx_r[i]);
        fscanf(ref, "%e", &wy_r[i]);
        fscanf(ref, "%e", &wz_r[i]);
        fscanf(ref, "%e", &u1_r[i]);
        fscanf(ref, "%e", &u2_r[i]);
        fscanf(ref, "%e", &u3_r[i]);     
        fscanf(ref, "%e", &u4_r[i]);    

        // Print to test
        //printf("%e  ", xx_r[i]);
        //printf("%e  ", xy_r[i]);
        //printf("%e  ", xz_r[i]);
        //printf("%e  ", vx_r[i]);
        //printf("%e  ", vy_r[i]);
        //printf("%e  ", vz_r[i]);
        //printf("%e  ", q2_r[i]);
        //printf("%e  ", q3_r[i]);
        //printf("%e  ", q4_r[i]);
        //printf("%e  ", wx_r[i]);
        //printf("%e  ", wy_r[i]);
        //printf("%e  ", wz_r[i]);
        //printf("%e  ", u1_r[i]);
        //printf("%e  ", u2_r[i]);
        //printf("%e  ", u3_r[i]);
        //printf("%e  ", u4_r[i]);
     }
    //int row_m = 10;
    //printf("\t mesure.txt file row %d: Value for xx = %.3e\n\n", row_m, xx_r[row_m]);
    //printf("\t mesure.txt file row %d: Value for xy = %.3e\n\n", row_m, xy_r[row_m]);
    //printf("\t mesure.txt file row %d: Value for xz = %.3e\n\n", row_m, xz_r[row_m]);
    // Close ref.txt file
    fclose(ref);

    //*******************************************************************************//
    //****************************** txt File Read End ****************************//
    //*******************************************************************************//


    while(up_smaple <= (3001 - (N+1))){
         acado_tic( &t );
    //**********************************************************************************//
    //****************************** Convert to array Start ****************************//
    //**********************************************************************************//

    int row_num  = up_smaple-1;
    numberArray_x0[0] = xx_m[row_num];
    numberArray_x0[1] = xy_m[row_num];
    numberArray_x0[2] = xz_m[row_num];
    numberArray_x0[3] = vx_m[row_num];
    numberArray_x0[4] = vy_m[row_num];
    numberArray_x0[5] = vz_m[row_num];       
    numberArray_x0[6] = q1_m[row_num];
    numberArray_x0[7] = q2_m[row_num];
    numberArray_x0[8] = q3_m[row_num];  
    numberArray_x0[9] = q4_m[row_num];    
    numberArray_x0[10] = wx_m[row_num]; 
    numberArray_x0[11] = wy_m[row_num]; 
    numberArray_x0[12] = wz_m[row_num]; 


    
    for(i = 0; i < N; i++){

    ref_numberArray_y[0 + (i*NY)] = xx_r[row_num + i];
    ref_numberArray_y[1 + (i*NY)] = xy_r[row_num + i];
    ref_numberArray_y[2 + (i*NY)] = xz_r[row_num + i];
    ref_numberArray_y[3 + (i*NY)] = vx_r[row_num + i];
    ref_numberArray_y[4 + (i*NY)] = vy_r[row_num + i];
    ref_numberArray_y[5 + (i*NY)] = vz_r[row_num + i];
    ref_numberArray_y[6 + (i*NY)] = q2_r[row_num + i];
    ref_numberArray_y[7 + (i*NY)] = q3_r[row_num + i];
    ref_numberArray_y[8 + (i*NY)] = q4_r[row_num + i];
    ref_numberArray_y[9 + (i*NY)] = wx_r[row_num + i];
    ref_numberArray_y[10 + (i*NY)] = wy_r[row_num + i];
    ref_numberArray_y[11 + (i*NY)] = wz_r[row_num + i];
    ref_numberArray_y[12 + (i*NY)] = u1_r[row_num + i];
    ref_numberArray_y[13 + (i*NY)] = u2_r[row_num + i];
    ref_numberArray_y[14 + (i*NY)] = u3_r[row_num + i];
    ref_numberArray_y[15 + (i*NY)] = u4_r[row_num + i];
    
    }
    ref_numberArray_yN[0] = xx_r[row_num + N];
    ref_numberArray_yN[1] = xy_r[row_num + N];
    ref_numberArray_yN[2] = xz_r[row_num + N];
    ref_numberArray_yN[3] = vx_r[row_num + N];
    ref_numberArray_yN[4] = vy_r[row_num + N];
    ref_numberArray_yN[5] = vz_r[row_num + N];
    ref_numberArray_yN[6] = q2_r[row_num + N];
    ref_numberArray_yN[7] = q3_r[row_num + N];
    ref_numberArray_yN[8] = q4_r[row_num + N];
    ref_numberArray_yN[9] = wx_r[row_num + N];
    ref_numberArray_yN[10] = wy_r[row_num + N];
    ref_numberArray_yN[11] = wz_r[row_num + N];

    //**********************************************************************************//
    //****************************** Convert to array End ******************************//
    //**********************************************************************************//

    
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
	// acado_tic( &t );

	/* The "real-time iterations" loop. */
	for(iter = 0; iter < NUM_STEPS; ++iter)
	{
        /* Perform the feedback step. */
		acado_feedbackStep( );

		/* Apply the new control immediately to the process, first NU components. */

		//if( VERBOSE ) printf("\tReal-Time Iteration %d:  KKT Tolerance = %.3e\n\n", iter, acado_getKKT() );

		/* Optional: shift the initialization (look at acado_common.h). */
        /* acado_shiftStates(2, 0, 0); */
		/* acado_shiftControls( 0 ); */
        // acado_printDifferentialVariables();
	    // acado_printControlVariables();

		/* Prepare for the next step. */
		acado_preparationStep();
	}
	/* Read the elapsed time. */
	real_t te = acado_toc( &t );

	//if( VERBOSE ) printf("\n\nEnd of the RTI loop. \n\n\n");

	/* Eye-candy. */

	if( !VERBOSE )
	
    //printf("\n\n Average time of one real-time iteration:   %.3g microseconds\n\n", 1e6 * te / NUM_STEPS);
    //printf("\n\n Average time of 10 real-time iteration:  %.3g seconds\n\n", te);


    //**************************************************************//
    //********************* Choosing ref. sample *******************//
    //**************************************************************//
    time_saver = time_saver + te;
    printf("\n\n Average time of 10 real-time iteration:  %.3g seconds\n\n", te);
    printf("\n\n Overall time:  %.3g seconds\n\n",time_saver);
    
    //while((dt_ref*jjj) < time_saver){
    
    //jjj = jjj+1;
    //}    

    
    //up_smaple = jjj+1;
    //low_smaple = jjj;
    //jjj = 0; 
    //printf("\n\n up_smaple:  %.3d \n\n", up_smaple);
    //printf("\n\n low_smaple:  %.3d \n\n", low_smaple);
    
    //**************************************************************//
    //********************* Choosing ref. sample *******************//
    //**************************************************************//
    up_smaple = up_smaple + 10;
    }


	acado_printDifferentialVariables();
	acado_printControlVariables();
    
    return 0;
}
