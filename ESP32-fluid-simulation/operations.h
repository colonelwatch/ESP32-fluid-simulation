#ifndef OPERATIONS_H
#define OPERATIONS_H

#include "Vector.h"
#include "Field.h"

// The below operations assume that the input and output have the same shape
// SCALAR_T and VECTOR_T are self-evident template args, but T means here that either a scalar or vector can be used

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
void semilagrangian_advect(Field<T> *new_property, const Field<T> *property, const Field<VECTOR_T> *velocity, float dt){
    int N_i = new_property->N_i, N_j = new_property->N_j;
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            // Get the source location, where we trace backwards and where +i-direction == -y-direction and 
            //  +j-direction == +x-direction
            VECTOR_T displacement = dt*velocity->index(i, j);
            VECTOR_T source = {j-displacement.x, i+displacement.y};

            // Clamp the source location within the boundaries
            if(source.y < -0.5f) source.y = -0.5f;
            if(source.y > N_i-0.5f) source.y = N_i-0.5f;
            if(source.x < -0.5f) source.x = -0.5f;
            if(source.x > N_j-0.5f) source.x = N_j-0.5f;

            // Get the source value with billinear interpolation
            int i11 = int(source.y), j11 = int(source.x), 
                i12 = i11, j12 = j11+1, 
                i21 = i11+1, j21 = j11, 
                i22 = i11+1, j22 = j11+1;
            float di = source.y-i11, dj = source.x-j11;
            T p11 = property->index(i11, j11), p12 = property->index(i12, j12),
                p21 = property->index(i21, j21), p22 = property->index(i22, j22);
            T interpolated = billinear_interpolate(di, dj, p11, p12, p21, p22);
            new_property->index(i, j) = interpolated;
        }
    }
    new_property->update_boundary();
}

template<class T>
void laplacian(Field<T> *del_dot_del_property, const Field<T> *property){
    int N_i = del_dot_del_property->N_i, N_j = del_dot_del_property->N_j;
    
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            T up, down, left, right, center;
            center = property->index(i, j);
            up = property->index(i-1, j);
            down = property->index(i+1, j);
            left = property->index(i, j-1);
            right = property->index(i, j+1);

            del_dot_del_property->index(i, j) = up+down+left+right-4*center;
        }
    }

    del_dot_del_property->update_boundary();
}

template<class SCALAR_T, class VECTOR_T>
void divergence(Field<SCALAR_T> *del_dot_velocity, const Field<VECTOR_T> *velocity){
    int N_i = del_dot_velocity->N_i, N_j = del_dot_velocity->N_j;
    
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            SCALAR_T upflow, downflow, leftflow, rightflow;
            downflow = -velocity->index(i+1, j).y; // we'll take positive as the inward direction
            upflow = velocity->index(i-1, j).y;
            leftflow = -velocity->index(i, j-1).x;
            rightflow = velocity->index(i, j+1).x;

            del_dot_velocity->index(i, j) = (upflow+downflow+leftflow+rightflow)/2;
        }
    }

    del_dot_velocity->update_boundary();
}

template<class SCALAR_T>
void gauss_seidel_pressure(Field<SCALAR_T> *pressure, const Field<SCALAR_T> *divergence, int iterations = 10){
    int N_i = pressure->N_i, N_j = pressure->N_j;

    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            pressure->index(i, j) = 0;
    
    pressure->update_boundary();

    for(int k = 0; k < iterations; k++){
        for(int i = 0; i < N_i; i++){
            for(int j = 0; j < N_j; j++){
                SCALAR_T divergence_center = divergence->index(i, j);
                SCALAR_T up, down, left, right;
                up = pressure->index(i-1, j);
                down = pressure->index(i+1, j);
                left = pressure->index(i, j-1);
                right = pressure->index(i, j+1);

                pressure->index(i, j) = (up+down+left+right-divergence_center)/4;
            }
        }

        pressure->update_boundary();
    }
}

template<class SCALAR_T, class VECTOR_T>
void gradient_and_subtract(Field<VECTOR_T> *velocity, const Field<SCALAR_T> *pressure){
    int N_i = velocity->N_i, N_j = velocity->N_j;
    
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            SCALAR_T up, down, left, right;
            up = pressure->index(i-1, j);
            down = pressure->index(i+1, j);
            left = pressure->index(i, j-1);
            right = pressure->index(i, j+1);

            velocity->index(i, j).x -= (right-left)/2;
            velocity->index(i, j).y -= (up-down)/2;
        }
    }

    velocity->update_boundary();
}

#endif