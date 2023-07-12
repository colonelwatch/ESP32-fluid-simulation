#ifndef OPERATIONS_H
#define OPERATIONS_H

#include "Vector.h"
#include "Field.h"

// The below operations assume that the input and output have the same shape

template<class T>
T billinear_interpolate(float di, float dj, T p11, T p12, T p21, T p22)
{
    T x1, x2, interpolated;
    x2 = p22*dj+p21*(1-dj);
    x1 = p12*dj+p11*(1-dj);
    interpolated = x2*di+x1*(1-di);
    return interpolated;
}

template<class T, class VECTOR_T>
void advect(Field<T> *output, const Field<T> *input, const Field<VECTOR_T> *velocity, float dt){
    for(int i = 0; i < output->N_i; i++){
        for(int j = 0; j < output->N_j; j++){
            // Get the source location
            VECTOR_T displacement = dt*velocity->index(i, j);
            VECTOR_T source = {j-displacement.x, i+displacement.y}; // tracing backwards where +i-direction == -y-direction and +j-direction == +x-direction

            // Clamp the source location within the boundaries
            if(source.y < -0.5) source.y = -0.5;
            if(source.y > output->N_i-0.5) source.y = output->N_i-0.5;
            if(source.x < -0.5) source.x = -0.5;
            if(source.x > output->N_j-0.5) source.x = output->N_j-0.5;

            // Get the source value with billinear interpolation
            int i11 = int(source.y), j11 = int(source.x), 
                i12 = i11, j12 = j11+1, 
                i21 = i11+1, j21 = j11, 
                i22 = i11+1, j22 = j11+1;
            float di = source.y-i11, dj = source.x-j11;
            T p11 = input->index(i11, j11), p12 = input->index(i12, j12),
                p21 = input->index(i21, j21), p22 = input->index(i22, j22);
            T interpolated = billinear_interpolate(di, dj, p11, p12, p21, p22);
            output->index(i, j) = interpolated;
        }
    }
    output->update_boundary();
}

template<class T>
void laplacian(Field<T> *output, const Field<T> *input){
    for(int i = 0; i < output->N_i; i++){
        for(int j = 0; j < output->N_j; j++){
            T up, down, left, right, center;
            center = input->index(i, j);
            up = input->index(i-1, j);
            down = input->index(i+1, j);
            left = input->index(i, j-1);
            right = input->index(i, j+1);

            output->index(i, j) = up+down+left+right-4*center;
        }
    }
    output->update_boundary();
}

template<class SCALAR_T, class VECTOR_T>
void divergence(Field<SCALAR_T> *output, const Field<VECTOR_T> *input){
    for(int i = 0; i < output->N_i; i++){
        for(int j = 0; j < output->N_j; j++){
            SCALAR_T upflow, downflow, leftflow, rightflow;
            downflow = -input->index(i+1, j).y;
            upflow = input->index(i-1, j).y;
            leftflow = -input->index(i, j-1).x;
            rightflow = input->index(i, j+1).x;

            output->index(i, j) = (upflow+downflow+leftflow+rightflow)/2;
        }
    }
    output->update_boundary();
}

template<class T>
void gauss_seidel_pressure(Field<T> *output, const Field<T> *input, int iterations = 10){
    int N_i = output->N_i, N_j = output->N_j;

    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            output->index(i, j) = 0;
    output->update_boundary();

    for(int k = 0; k < iterations; k++){
        for(int i = 0; i < N_i; i++){
            for(int j = 0; j < N_j; j++){
                T divergence = input->index(i, j);
                T up, down, left, right;
                up = output->index(i-1, j);
                down = output->index(i+1, j);
                left = output->index(i, j-1);
                right = output->index(i, j+1);

                output->index(i, j) = (up+down+left+right-divergence)/4;
            }
        }
        output->update_boundary();
    }
}

template<class SCALAR_T, class VECTOR_T>
void gradient_and_subtract(Field<VECTOR_T> *output, const Field<SCALAR_T> *input){
    for(int i = 0; i < output->N_i; i++){
        for(int j = 0; j < output->N_j; j++){
            SCALAR_T up, down, left, right;
            up = input->index(i-1, j);
            down = input->index(i+1, j);
            left = input->index(i, j-1);
            right = input->index(i, j+1);

            output->index(i, j).x -= (right-left)/2;
            output->index(i, j).y -= (up-down)/2;
        }
    }
    output->update_boundary();
}

#endif