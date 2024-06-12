#ifndef OPERATIONS_H
#define OPERATIONS_H

#include "Vector.h"
#include "Field.h"

#define FLOOR(x) ( x > 0 || x == int(x) ? int(x) : int(x)-1 )

// The below operations assume that the input and output have the same shape
// SCALAR_T and VECTOR_T are self-evident template args, but T means here that either a scalar or vector can be used

template<class T>
T billinear_interpolate(float di, float dj, T p11, T p12, T p21, T p22)
{
    T x1, x2, interpolated;
    x1 = p11*(1-dj)+p12*dj; // interp between lower-left and upper-left
    x2 = p21*(1-dj)+p22*dj; // interp between lower-right and upper-right
    interpolated = x1*(1-di)+x2*di; // interp between left and right
    return interpolated;
}

template<class T, class VECTOR_T>
void semilagrangian_advect(Field<T> *new_property, const Field<T> *property, const Field<VECTOR_T> *velocity, float dt){
    int N_i = new_property->N_i, N_j = new_property->N_j;
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            VECTOR_T displacement = dt*velocity->index(i, j);
            VECTOR_T source = {i-displacement.x, j-displacement.y};

            // Clamp the source location within the boundaries
            if(source.x < -0.5f) source.x = -0.5f;
            if(source.x > N_i-0.5f) source.x = N_i-0.5f;
            if(source.y < -0.5f) source.y = -0.5f;
            if(source.y > N_j-0.5f) source.y = N_j-0.5f;

            // Get the source value with billinear interpolation
            int i11 = FLOOR(source.x), j11 = FLOOR(source.y), 
                i12 = i11, j12 = j11+1, 
                i21 = i11+1, j21 = j11, 
                i22 = i11+1, j22 = j11+1;
            float di = source.x-i11, dj = source.y-j11;
            T p11 = property->index(i11, j11), p12 = property->index(i12, j12),
                p21 = property->index(i21, j21), p22 = property->index(i22, j22);
            T interpolated = billinear_interpolate(di, dj, p11, p12, p21, p22);
            new_property->index(i, j) = interpolated;
        }
    }
    new_property->update_boundary();
}

template<class SCALAR_T, class VECTOR_T>
void divergence(Field<SCALAR_T> *del_dot_velocity, const Field<VECTOR_T> *velocity){
    int N_i = del_dot_velocity->N_i, N_j = del_dot_velocity->N_j;
    
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            SCALAR_T leftflow, rightflow, downflow, upflow;
            leftflow = -velocity->index(i-1, j).x;
            rightflow = velocity->index(i+1, j).x;
            downflow = -velocity->index(i, j-1).y;
            upflow = velocity->index(i, j+1).y;

            del_dot_velocity->index(i, j) = (upflow+downflow+leftflow+rightflow)/2;
        }
    }

    del_dot_velocity->update_boundary();
}

template<class SCALAR_T>
void sor_pressure(Field<SCALAR_T> *pressure, const Field<SCALAR_T> *divergence, int iterations, float omega){
    int N_i = pressure->N_i, N_j = pressure->N_j;

    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            pressure->index(i, j) = 0;
    
    pressure->update_boundary();

    for(int k = 0; k < iterations; k++){
        for(int kk = 0; kk <= 1; kk++){
            for(int i = 0, j_0 = kk; i < N_i; i++, j_0 ^= 1){
                for(int j = j_0; j < N_j; j += 2){
                    SCALAR_T div = divergence->index(i, j);
                    SCALAR_T left, right, down, up;
                    left = pressure->index(i-1, j);
                    right = pressure->index(i+1, j);
                    down = pressure->index(i, j-1);
                    up = pressure->index(i, j+1);

                    pressure->index(i, j) = (1-omega)*pressure->index(i, j) + omega*(div-left-right-down-up)/(-4);
                }
            }

            pressure->update_boundary();
        }
    }
}

template<class SCALAR_T, class VECTOR_T>
void gradient_and_subtract(Field<VECTOR_T> *velocity, const Field<SCALAR_T> *pressure){
    int N_i = velocity->N_i, N_j = velocity->N_j;
    
    for(int i = 0; i < N_i; i++){
        for(int j = 0; j < N_j; j++){
            SCALAR_T left, right, down, up;
            left = pressure->index(i-1, j);
            right = pressure->index(i+1, j);
            down = pressure->index(i, j-1);
            up = pressure->index(i, j+1);

            velocity->index(i, j).x -= (right-left)/2;
            velocity->index(i, j).y -= (up-down)/2;
        }
    }

    velocity->update_boundary();
}

#endif