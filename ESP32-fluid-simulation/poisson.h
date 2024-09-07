#ifndef POIS_H
#define POIS_H

#include "operations.h"

void poisson_solve(float *p, float *div, int dim_x, int dim_y, float dx,
        int iters, float omega);

#endif