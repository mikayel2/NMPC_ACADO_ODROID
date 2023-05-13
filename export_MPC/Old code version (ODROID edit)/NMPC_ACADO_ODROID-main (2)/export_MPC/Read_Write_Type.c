/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

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

main()
{
    // FILE object - https://www.tutorialspoint.com/cprogramming/c_file_io.htm
    FILE *myFile;
    myFile = fopen("ref.txt", "r"); // "r" means for reading
    
    // Check whether file is empty or not
    if (myFile == NULL) {
        printf("Error: Unable to open the file.\n");
        return 1;
    }
    
    int rows = 17; 
    int cols = 18;
    float numberArray[rows][cols]; // 18 columns and 17 rows to read
    int i,j;
    
    
    for (i = 0; i < rows; i++){ // rows
    for (j = 0; j < cols; j++){ // columns
    
        //read file into array
        // fscanf function - https://www.tutorialspoint.com/c_standard_library/c_function_fscanf.htm
        fscanf(myFile, "%e", &numberArray[i][j]);
    }
    }
    // Close the file
    fclose(myFile);
    
    // Print for test
    for (i = 0; i < rows; i++){ // rows
    for (j = 0; j < cols; j++)  // columns
    {
        printf("%e  ", numberArray[i][j]);
        
    }
        printf(" \n", numberArray[i][j]);
        
    }
    
    // Write in txt file
    FILE *output_file;
    output_file = fopen("output.txt", "w"); // "w" means write
    
    if (output_file == NULL) {
            printf("Error: Unable to open the output file.\n");
            return 1;
        }
    
    // Write the data to the output file
    for (i = 0; i < rows; i++) {
    for (j = 0; j < cols; j++) {
        
        fprintf(output_file, "%e    ", numberArray[i][j]);
        }
        fprintf(output_file, "  \n");
        }
     
    
    // Type test 
    PRINT_VAR_TYPE(numberArray[i][j]);
    
    // Close the output file
    fclose(output_file);


return 0;
}