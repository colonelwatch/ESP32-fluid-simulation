// Simulates circle of color in a rotating tilted box and outputs the velocity, pressure, divergence, and color

#include <cmath>
#include <fstream>
#include <iomanip>

#include "../ESP32-fluid-simulation/Vector.h"
#include "../ESP32-fluid-simulation/Field.h"
#include "../ESP32-fluid-simulation/operations.h"

#define N_ROWS 64
#define N_COLS 64
#define SECONDS 10
#define DT 0.001
#define OUTPUT_FPS 60

typedef Vector<float> FloatVector;

int main(){
    // Initialize the color field
    int center_i = N_ROWS/2, center_j = N_COLS/2;
    float color_arr[N_ROWS][N_COLS];
    for(int i = 0; i < N_ROWS; i++){
        for(int j = 0; j < N_COLS; j++){
            float distance = sqrt(pow(i-center_i, 2) + pow(j-center_j, 2));
            if(distance < 8) color_arr[i][j] = 1;
            else color_arr[i][j] = 0;
        }
    }
    Field<float, CLONE> color_field(N_ROWS, N_COLS);
    color_field = (float*)color_arr;

    // Initialize the velocity field
    FloatVector zero_arr[N_ROWS][N_COLS] = {0};
    Field<FloatVector, NEGATIVE> velocity_field(N_ROWS, N_COLS);
    velocity_field = (FloatVector*)zero_arr;

    // Declare the other fields
    Field<FloatVector, NEGATIVE> temp_vector_field(N_ROWS, N_COLS);
    Field<float, CLONE> temp_scalar_field(N_ROWS, N_COLS);
    Field<float, CLONE> pressure_field(N_ROWS, N_COLS);

    #ifndef NO_FILE_OUTPUT
    // Open files for output
    std::ofstream velocity_file("velocity.txt"), 
                  pressure_file("pressure.txt"), 
                  divergence_file("divergence.txt"),
                  color_file("color.txt");
    #endif
    
    const int total_timesteps = SECONDS/DT;
    const int timesteps_per_frame = 1/(OUTPUT_FPS*DT);

    for(int i = 0; i < total_timesteps; i++){
        // Advect the velocity field
        advect(&temp_vector_field, &velocity_field, &velocity_field, DT);
        velocity_field = temp_vector_field;

        // Apply a force in the center of the velocity field for a little time
        if(i < 0.1/DT){
            const int center_i = N_ROWS/2, center_j = N_COLS/2;
            FloatVector dv = FloatVector({0, 10});
            velocity_field.index(center_i, center_j) += dv;
            velocity_field.index(center_i+1, center_j) += dv;
            velocity_field.index(center_i, center_j+1) += dv;
            velocity_field.index(center_i+1, center_j+1) += dv;
        }

        // Zero out the divergence of the velocity field
        jacobi_pressure(&pressure_field, &velocity_field);
        gradient_and_subtract(&velocity_field, &pressure_field);

        // Advect the color field
        advect(&temp_scalar_field, &color_field, &velocity_field, DT);
        color_field = temp_scalar_field;

        #ifndef NO_FILE_OUTPUT
        if(i % timesteps_per_frame == 0){
            // Output velocity and pressure fields
            velocity_file << velocity_field.toString(2) << "\n\n";
            pressure_file << pressure_field.toString(2) << "\n\n";

            // Calculate and output the divergence of the velocity field
            divergence(&temp_scalar_field, &velocity_field);
            divergence_file << temp_scalar_field.toString(2) << "\n\n";

            // Output the color field
            color_file << color_field.toString(2) << "\n\n";
        }
        #endif
    }

    #ifndef NO_FILE_OUTPUT
    // Close the files
    velocity_file.close();
    pressure_file.close();
    divergence_file.close();
    color_file.close();
    #endif

    std::cout << "Simulation done!" << std::endl;
        
    return 0;
}