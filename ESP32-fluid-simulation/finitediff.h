#ifndef FINITE_DIFF_H
#define FINITE_DIFF_H

#include "vector.h"
#include "operations.h"

void divergence(float *div, Vector<float> *v, int dim_x, int dim_y, float dx);

void subtract_gradient(Vector<float> *v, float *p, int dim_x, int dim_y,
        float dx);

#endif