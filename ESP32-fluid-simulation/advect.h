#ifndef ADVECT_H
#define ADVECT_H

#include <utility>
#include <cmath>

#include "vector.h"
#include "operations.h"

template<typename T>
using TPromoted = decltype(std::declval<T>()*std::declval<float>());

template <class T>
static TPromoted<T> lerp(float di, T p1, T p2) {
    return p1 * (1 - di) + p2 * di;
}

template <class T>
static TPromoted<T> billinear_interpolate(float di, float dj, T p11, T p12,
                                          T p21, T p22) {
    return lerp(di, lerp(dj, p11, p12), lerp(dj, p21, p22));
}

template <class T>
static T sample(T *p, float i, float j, int dim_x, int dim_y, bool no_slip) {
    bool x_under = i < 0;
    bool x_over = i >= dim_x - 1;
    bool y_under = j < 0;
    bool y_over = j >= dim_y - 1;

    bool x_oob = x_under || x_over;
    bool y_oob = y_under || y_over;

    float i_floor = floorf(i), j_floor = floorf(j);
    float di = i - i_floor, dj = j - j_floor;
    int ij;

    if (!x_oob && !y_oob) {   // typical case: not near the boundary
        ij = index(i_floor, j_floor, dim_x);
        return billinear_interpolate(di, dj, p[ij], p[ij + dim_x], p[ij + 1],
                                     p[ij + dim_x + 1]);
    }

    // interpolate along the boundary
    T p_edge;
    if (x_oob && y_oob) {  // on a corner
        ij = index((x_under? 0 : dim_x - 1), (y_under? 0 : dim_y - 1), dim_x);
        p_edge = p[ij];
    } else if (x_oob) {  // on left or right boundary
        ij = index((x_under? 0 : dim_x - 1), j_floor, dim_x);
        p_edge = lerp(dj, p[ij], p[ij + dim_x]);
    } else {  // y_oob, on bottom or top boundary
        ij = index(i_floor, (y_under? 0 : dim_y - 1), dim_x);
        p_edge = lerp(di, p[ij], p[ij + 1]);
    }

    if (!no_slip) {
        return p_edge;
    }

    // apply discount to implement no-slip, with zero at the boundary and beyond
    float overshoot_factor = 1.0f;
    if (x_oob) {
        float overshoot_x = x_under ? -i : i - (dim_x - 1);
        overshoot_factor *= overshoot_x < 0.5 ? (1 - 2 * overshoot_x) : 0;
    }
    if (y_oob) { 
        float overshoot_y = y_under ? -j : j - (dim_y - 1);
        overshoot_factor *= overshoot_y < 0.5 ? (1 - 2 * overshoot_y) : 0;
    }
    return overshoot_factor * p_edge;
}

template <class T, class U>
void advect(T *next_p, T *p, Vector2<U> *vel, int dim_x, int dim_y, float dt,
            bool no_slip)
{
    for (int i = 0; i < dim_x; i++) {
        for (int j = 0; j < dim_y; j++) {
            int ij = index(i, j, dim_x);
            Vector2<float> source = Vector2<float>(i, j) - vel[ij] * dt;
            next_p[ij] = sample(p, source.x, source.y, dim_x, dim_y, no_slip);
        }
    }
}

#endif
