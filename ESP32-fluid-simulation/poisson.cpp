#include "poisson.h"
#include "operations.h"

struct pois_context {
    float *d;
    float dx;
    float omega;
};

static inline int point_is_red(int i, int j) {
    return (i + j) & 0x1;
}

template<class T, class U>
static void domain_iter_red_black(U (*expr_safe)(T*, int, int, int, int, void*),
    U (*expr_fast)(T*, int, int, int, int, void*), U *wrt, T *rd, int dim_x,
    int dim_y, void *ctx)
{
    int i_max = (dim_x)-1, j_max = (dim_y)-1;  // inclusive!
    int ij, offset;

    int bottom_left_is_red = point_is_red(0, 0),
        bottom_right_is_red = point_is_red(i_max, 0),
        top_left_is_red = point_is_red(0, j_max);

    int on_red = 0;
    repeat_on_red:  // on arrival to this label, on_red = 1

    // Loop over the main body (starting from 1,1 as black or 2,1 as red)
    for (int j = 1, offset = on_red; j < j_max; ++j, offset ^= 1) {
        for (int i = 1+offset; i < i_max; i += 2) {
            ij = index(i, j, dim_x);
            wrt[ij] = expr_fast(&rd[ij], i, j, dim_x, dim_y, ctx);
        }
    }

    // Loop over the bottom (including left and right corners)
    offset = (on_red == bottom_left_is_red) ? 0 : 1;
    for(int i = offset; i <= i_max; i += 2) {
        ij = index(i, 0, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i, 0, dim_x, dim_y, ctx);
    }

    // Loop over the top (including left and right corners)
    offset = (on_red == top_left_is_red)? 0 : 1;
    for(int i = offset; i <= i_max; i += 2) {
        ij = index(i, j_max, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i, j_max, dim_x, dim_y, ctx);
    }

    // Loop over the left (starting from 0,1 or 0,2)
    offset = (on_red == !bottom_left_is_red) ? 1 : 2;  // we're *adjacent* it
    for(int j = offset; j < j_max; j += 2) {
        ij = index(0, j, dim_x);
        wrt[ij] = expr_safe(&rd[ij], 0, j, dim_x, dim_y, ctx);
    }

    // Loop over the right (starting from i_max,1 or i_max,2)
    offset = (on_red == !bottom_right_is_red)? 1 : 2;
    for(int j = offset; j < j_max; j += 2) {
        ij = index(i_max, j, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i_max, j, dim_x, dim_y, ctx);
    }

    if (!on_red) {
        on_red = 1;
        goto repeat_on_red;
    }
}

static float pois_expr_safe(float *p, int i, int j, int dim_x, int dim_y,
        void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    int i_max = dim_x-1, j_max = dim_y-1;

    float p_sum = 0;
    int a_ii = 0;
    if (i > 0) {
        p_sum += *(p-1);
        ++a_ii;
    }
    if (i < i_max) {
        p_sum += *(p+1);
        ++a_ii;
    }
    if (j > 0) {
        p_sum += *(p-dim_x);
        ++a_ii;
    }
    if (j < j_max) {
        p_sum += *(p+dim_x);
        ++a_ii;
    }

    static const float neg_a_ii_inv[5] = {0, 0, -1.0/2.0, -1.0/3.0, -1.0/4.0};
    int ij = index(i, j, dim_x);

    return neg_a_ii_inv[a_ii] * (pois_ctx->dx * pois_ctx->d[ij] - p_sum);
}

// static float pois_expr_fast(float *p, int i, int j, int dim_x, int dim_y,
//         void *ctx)
// {
//     struct pois_context *pois_ctx = (struct pois_context*)ctx;

//     float p_sum = ((*(p-1) + *(p+1)) + (*(p-dim_x) + *(p+dim_x)));

//     int ij = index(i, j, dim_x);
//     return -0.25f * (pois_ctx->dx * pois_ctx->d[ij] - p_sum);
// }

static float pois_sor_safe(float *p, int i, int j, int dim_x, int dim_y,
        void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    float omega = pois_ctx->omega;
    float p_gs = pois_expr_safe(p, i, j, dim_x, dim_y, ctx);
    return (1-omega)*(*p) + omega*p_gs;
}

static float pois_sor_fast(float *p, int i, int j, int dim_x, int dim_y,
        void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    float omega = pois_ctx->omega;

    float p_sum = (*(p-1) + *(p+1)) + (*(p-dim_x) + *(p+dim_x));

    int ij = index(i, j, dim_x);
    float p_gs = -0.25f * (pois_ctx->dx * pois_ctx->d[ij] - p_sum);

    return (1-omega)*(*p) + omega*p_gs;
}

void poisson_solve(float *p, float *div, int dim_x, int dim_y, float dx,
        int iters, float omega)
{
    for (int ij = 0; ij < dim_x*dim_y; ij++) {
        p[ij] = 0;
    }
    struct pois_context pois_ctx = {.d = div, .dx = dx, .omega = omega};
    for (int k = 0; k < iters; k++) {
        domain_iter_red_black(pois_sor_safe, pois_sor_fast, p, p, dim_x, dim_y,
            &pois_ctx);
    }
}
