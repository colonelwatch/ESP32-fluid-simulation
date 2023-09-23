// Simulates circle of color in a rotating tilted box and outputs the velocity, pressure, divergence, and color

#include <cmath>
#include <fstream>
#include <iomanip>

#include "../ESP32-fluid-simulation/Vector.h"
#include "../ESP32-fluid-simulation/Field.h"
#include "../ESP32-fluid-simulation/operations.h"

#define N 64 // width and height of the simulation
#define SECONDS 10
#define DT 0.001
#define OUTPUT_FPS 60

typedef Vector<float> FloatVector;

int main(){
    // Initialize the color field
    int center_i = N/2, center_j = N/2;
    float color_arr[N][N];
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            float distance = sqrt(pow(i-center_i, 2) + pow(j-center_j, 2));
            if(distance < 8) color_arr[i][j] = 1;
            else color_arr[i][j] = 0;
        }
    }
    Field<float> color_field(N, N, CLONE);
    color_field = (float*)color_arr;

    // Initialize the velocity field
    FloatVector zero_arr[N][N] = {0};
    Field<FloatVector> velocity_field(N, N, NEGATIVE);
    velocity_field = (FloatVector*)zero_arr;

    // Declare the other fields
    Field<FloatVector> temp_vector_field(N, N, NEGATIVE);
    Field<float> temp_scalar_field(N, N, CLONE);
    Field<float> pressure_field(N, N, CLONE);

    #ifndef NO_FILE_OUTPUT
    // Open files for output
    std::ofstream velocity_file("sim_velocity.txt"), 
                  divergence_file("sim_divergence.txt"),
                  color_file("sim_color.txt");
    #endif
    
    const int total_timesteps = SECONDS/DT;
    const int timesteps_per_frame = 1/(OUTPUT_FPS*DT);

    for(int i = 0; i < total_timesteps; i++){
        // Advect the velocity field
        semilagrangian_advect(&temp_vector_field, &velocity_field, &velocity_field, DT);
        velocity_field = temp_vector_field;

        // Apply a force in the center of the velocity field for a little time
        if(i < 0.1/DT){
            const int center_i = N/2, center_j = N/2;
            FloatVector dv = FloatVector({-10, 0});
            velocity_field.index(center_i, center_j) += dv;
            velocity_field.index(center_i+1, center_j) += dv;
            velocity_field.index(center_i, center_j+1) += dv;
            velocity_field.index(center_i+1, center_j+1) += dv;
        }

        // Zero out the divergence of the velocity field
        const float sor_omega = 1.90; // 1.0 reverts SOR to Gauss-Seidel, but 2/(1+sin(pi/60)) = 1.90 is optimal?
        divergence(&temp_scalar_field, &velocity_field);
        sor_pressure(&pressure_field, &temp_scalar_field, 10, sor_omega);
        gradient_and_subtract(&velocity_field, &pressure_field);

        // Advect the color field
        semilagrangian_advect(&temp_scalar_field, &color_field, &velocity_field, DT);
        color_field = temp_scalar_field;

        #ifndef NO_FILE_OUTPUT
        if(i % timesteps_per_frame == 0){
            // Output velocity and pressure fields
            velocity_file << velocity_field.toString(2) << "\n\n";

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
    divergence_file.close();
    color_file.close();

    std::ofstream sim_params("sim_params.json");
    sim_params << "{\n";
    sim_params << "    \"N\": " << N << ",\n";
    sim_params << "    \"SECONDS\": " << SECONDS << ",\n";
    sim_params << "    \"DT\": " << DT << ",\n";
    sim_params << "    \"OUTPUT_FPS\": " << OUTPUT_FPS << "\n";
    sim_params << "}";
    sim_params.close();
    #endif

    std::cout << "Simulation done!" << std::endl;
        
    return 0;
}