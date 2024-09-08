// Simulates circle of color in a rotating tilted box and outputs the velocity, pressure, divergence, and color

#include <cmath>
#include <fstream>
#include <iomanip>

#include "../ESP32-fluid-simulation/Vector2.h"
#include "../ESP32-fluid-simulation/Field.h"
#include "../ESP32-fluid-simulation/operations.h"

#define N 64 // width and height of the simulation
#define SECONDS 10
#define DT 0.001
#define OUTPUT_FPS 60

#define SWAP(x, y) do { auto temp = x; x = y; y = temp; } while(0);

int main(){
    // Initialize the color field
    Field<float> *color_field = new Field<float>(N, N, CLONE);
    int center_i = N/2, center_j = N/2;
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            float distance = sqrt(pow(i-center_i, 2) + pow(j-center_j, 2));
            if(distance < 8) color_field->index(i, j) = 1;
            else color_field->index(i, j) = 0;
        }
    }
    color_field->update_boundary();

    // Initialize the velocity field and declare a temporary vector field
    Field<Vector2<float>> *velocity_field = new Field<Vector2<float>>(N, N, NEGATIVE),
                         *temp_vector_field = new Field<Vector2<float>>(N, N, NEGATIVE);
    for(int i = 0; i < N; i++)
        for(int j = 0; j < N; j++)
            velocity_field->index(i, j) = {0, 0};
    velocity_field->update_boundary();

    // Declare the pressure field and a temporary scalar field
    Field<float> *pressure_field = new Field<float>(N, N, CLONE),
                 *temp_scalar_field = new Field<float>(N, N, CLONE);

    #ifndef NO_FILE_OUTPUT
    // Open files for output
    std::ofstream velocity_file("sim_velocity.arr", std::ios::out | std::ios::binary), 
                  divergence_file("sim_divergence.arr", std::ios::out | std::ios::binary), 
                  color_file("sim_color.arr", std::ios::out | std::ios::binary);
    #endif
    
    const int total_timesteps = SECONDS/DT;
    const int timesteps_per_frame = 1/(OUTPUT_FPS*DT);

    for(int i = 0; i < total_timesteps; i++){
        // Advect the velocity field
        semilagrangian_advect(temp_vector_field, velocity_field, velocity_field, DT);
        SWAP(temp_vector_field, velocity_field);

        // Apply a force in the center of the velocity field for a little time
        if(i < 0.1/DT){
            const int center_i = N/2, center_j = N/2;
            Vector2<float> dv = Vector2<float>({-10, 0});
            velocity_field->index(center_i, center_j) += dv;
            velocity_field->index(center_i+1, center_j) += dv;
            velocity_field->index(center_i, center_j+1) += dv;
            velocity_field->index(center_i+1, center_j+1) += dv;
        }

        // Zero out the divergence of the velocity field
        divergence(temp_scalar_field, velocity_field);
        sor_pressure(pressure_field, temp_scalar_field, 10, 1.96);
        gradient_and_subtract(velocity_field, pressure_field);

        // Advect the color field
        semilagrangian_advect(temp_scalar_field, color_field, velocity_field, DT);
        SWAP(color_field, temp_scalar_field);

        #ifndef NO_FILE_OUTPUT
        if(i % timesteps_per_frame == 0){
            int n_bytes;
            char *bytes;

            // Output the velocity field
            bytes = velocity_field->as_bytes(&n_bytes);
            velocity_file.write(bytes, n_bytes);
            delete[] bytes;

            // Calculate and output the divergence of the velocity field
            divergence(temp_scalar_field, velocity_field);
            bytes = temp_scalar_field->as_bytes(&n_bytes);
            divergence_file.write(bytes, n_bytes);
            delete[] bytes;

            // Output the color field
            bytes = color_field->as_bytes(&n_bytes);
            color_file.write(bytes, n_bytes);
            delete[] bytes;
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