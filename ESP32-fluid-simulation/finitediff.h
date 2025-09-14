#ifndef FINITE_DIFF_H
#define FINITE_DIFF_H

#include "vector.h"

void calculate_divergence(float *div, Vector2<float> *v, int dim_x, int dim_y,
                          float dx);

void subtract_gradient(Vector2<float> *v, float *p, int dim_x, int dim_y,
                       float dx);

#endif
