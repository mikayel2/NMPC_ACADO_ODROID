/******************************************************************************

                            Online C Compiler.
                Code, Compile, Run and Debug C program online.
Write your code in this editor and press "Run" button to compile and execute it.

*******************************************************************************/

#include <stdio.h>

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
    //read file into array
    int rows = 17;
    int cols = 18;
    float numberArray[rows][cols]; // 18 columns and 17 rows to read
    int i,j;
    
    
    for (i = 0; i < rows; i++){ // rows
    for (j = 0; j < cols; j++){ // columns
    
        fscanf(myFile, "%e", &numberArray[i][j]);
    }
    }
    // Close the file
    fclose(myFile);
    
    // Print for test
    for (i = 0; i < rows; i++){ // rows
    for (i = 0; i < cols; i++)  // columns
    {
        //printf("Number is: %E\n\n", numberArray[j][i]);
        printf("%e\n\n", numberArray[j][i]);
        
    }}

return 0;
}