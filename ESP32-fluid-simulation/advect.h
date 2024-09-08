#ifndef ADVECT_H
#define ADVECT_H

#include "vector.h"
#include "operations.h"

static inline int floor(int x) {
    return x > 0 || x == int(x) ? int(x) : int(x) - 1;
}

template <class T>
static inline TPromoted<T> lerp(float di, T p1, T p2) {
    return p1 * (1 - di) + p2 * di;
}

template <class T>
static TPromoted<T> billinear_interpolate(float di, float dj, T p11, T p12, T p21, T p22) {
    return lerp(di, lerp(dj, p11, p12), lerp(dj, p21, p22));
}

template <class T>
static T sample(T *p, float i, float j, int dim_x, int dim_y, int no_slip) {
    int overshot_in_x = (i >= dim_x-1 || i < 0);
    int overshot_in_y = (j >= dim_y-1 || j < 0);

    if (!overshot_in_x && !overshot_in_y) {
        int i_floor = (int)i, j_floor = (int)j;
        float di = i - i_floor, dj = j - j_floor;
        int ij = index(i_floor, j_floor, dim_x);
        return billinear_interpolate(di, dj, p[ij], p[ij + dim_x], p[ij + 1],
            p[ij + dim_x + 1]);
    }

    float overshoot_right = i - (dim_x-1), overshoot_left = 0 - i;
    float overshoot_up = j - (dim_y-1), overshoot_down = 0 - j;
    if (overshoot_right > 0.5f) {
        overshoot_right = 0.5f;
        i = dim_x - 0.5f;
    }
    if (overshoot_left > 0.5f) {
        overshoot_left = 0.5f;
        i = -0.5f;
    }
    if (overshoot_up > 0.5f) {
        overshoot_up = 0.5f;
        j = dim_y - 0.5f;
    }
    if (overshoot_down > 0.5f) {
        overshoot_down = 0.5f;
        j = -0.5f;
    }

    int i_floor = floor(i), j_floor = floor(j);
    float di = i - i_floor, dj = j - j_floor;
    int ij = index(i_floor, j_floor, dim_x);

    T p_edge;
    float overshoot_factor;
    if (overshot_in_x && overshot_in_y) {
        int offset = 0;
        if (overshoot_right >= 0) {
            overshoot_factor = (1 - 2 * overshoot_right);
        } else { // overshoot_left > 0
            offset += 1;
            overshoot_factor = (1 - 2 * overshoot_left);
        }
        if (overshoot_up >= 0) {
            overshoot_factor *= (1 - 2 * overshoot_up);
        } else { // overshoot_down > 0
            offset += dim_x;
            overshoot_factor *= (1 - 2 * overshoot_down);
        }
        p_edge = p[ij + offset];
    } else if (overshot_in_x) {
        if (overshoot_right >= 0) {
            p_edge = lerp(dj, p[ij], p[ij + dim_x]);
            overshoot_factor = 1 - 2 * overshoot_right;
        } else { // overshoot_left > 0
            p_edge = lerp(dj, p[ij + 1], p[ij + dim_x + 1]);
            overshoot_factor = 1 - 2 * overshoot_left;
        }
    } else { // overshot_in_y
        if (overshoot_up >= 0) {
            p_edge = lerp(di, p[ij], p[ij + 1]);
            overshoot_factor = 1 - 2 * overshoot_up;
        } else { // overshoot_down > 0
            p_edge = lerp(di, p[ij + dim_x], p[ij + dim_x + 1]);
            overshoot_factor = 1 - 2 * overshoot_down;
        }
    }

    if (no_slip) {
        return p_edge * overshoot_factor;
    } else {
        return p_edge;
    }
}

template <class T, class U>
void advect(T *next_p, T *p, Vector2<U> *vel, int dim_x, int dim_y, float dt,
        int no_slip)
{
    for (int i = 0; i < dim_x; i++) {
        for (int j = 0; j < dim_y; j++) {
            int ij = index(i, j, dim_x);
            Vector2<U> vel_ij = vel[ij];
            Vector2<U> source = {i-vel_ij.x*dt, j-vel_ij.y*dt};
            next_p[ij] = sample(p, source.x, source.y, dim_x, dim_y, no_slip);
        }
    }
}

#endif