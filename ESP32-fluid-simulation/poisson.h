#ifndef POIS_H
#define POIS_H


void poisson_solve(float *p, float *div, int dim_x, int dim_y, float dx,
        int iters, float omega);

#endif