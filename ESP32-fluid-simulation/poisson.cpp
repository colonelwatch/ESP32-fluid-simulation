#include "poisson.h"
#include "operations.h"

struct pois_context {
    float *d;
    float dx;
    float omega;
};

static inline bool is_red(int i, int j) {
    return (i + j) & 0x1;
}

template<class T, class U>
static void domain_iter_red_black(kernel_func_t<T, U> expr_safe,
                                  kernel_func_t<T, U> expr_fast, U *wrt, T *rd,
                                  int dim_x, int dim_y, void *ctx)
{
    int i_max = dim_x - 1, j_max = dim_y - 1;
    int ij;

    bool on_red = false;
    repeat_on_red:  // on arrival to this label, on_red = true

    // Main body, starting from (1,1) or (2,1)
    for (int j = 1; j < j_max; ++j) {
        for (int i = (on_red == is_red(1, j) ? 1 : 2); i < i_max; i += 2) {
            ij = index(i, j, dim_x);
            wrt[ij] = expr_fast(&rd[ij], i, j, dim_x, dim_y, ctx);
        }
    }

    // Bottom row incl. left & right corners, starting from (0,0) or (1,0)
    for(int i = (on_red == is_red(0, 0) ? 0 : 1); i <= i_max; i += 2) {
        ij = index(i, 0, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i, 0, dim_x, dim_y, ctx);
    }

    // Top row incl. left & right corners, starting from (0,j_max) or (1,j_max)
    for(int i = (on_red == is_red(0, j_max) ? 0 : 1); i <= i_max; i += 2) {
        ij = index(i, j_max, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i, j_max, dim_x, dim_y, ctx);
    }

    // Leftmost column, starting from (0,1) or (0,2)
    for(int j = (on_red == is_red(0, 1) ? 1 : 2); j < j_max; j += 2) {
        ij = index(0, j, dim_x);
        wrt[ij] = expr_safe(&rd[ij], 0, j, dim_x, dim_y, ctx);
    }

    // Rightmost column, starting from (i_max,1) or (i_max,2)
    for(int j = (on_red == is_red(i_max, 1) ? 1 : 2); j < j_max; j += 2) {
        ij = index(i_max, j, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i_max, j, dim_x, dim_y, ctx);
    }

    if (!on_red) {
        on_red = true;
        goto repeat_on_red;
    }
}

static float pois_gs_safe(float *p, int i, int j, int dim_x, int dim_y,
                          void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    static const float neg_a_ii_inv[5] = {0, 0, -1.0/2.0, -1.0/3.0, -1.0/4.0};

    float p_sum = 0;
    int a_ii = 0;
    if (i > 0) {
        p_sum += *(p - 1);
        ++a_ii;
    }
    if (i < dim_x - 1) {
        p_sum += *(p + 1);
        ++a_ii;
    }
    if (j > 0) {
        p_sum += *(p - dim_x);
        ++a_ii;
    }
    if (j < dim_y - 1) {
        p_sum += *(p + dim_x);
        ++a_ii;
    }

    float d_ij = pois_ctx->d[index(i, j, dim_x)];
    return neg_a_ii_inv[a_ii] * (pois_ctx->dx * d_ij - p_sum);
}

static float pois_sor_safe(float *p, int i, int j, int dim_x, int dim_y,
                           void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    float omega = pois_ctx->omega;
    float p_gs = pois_gs_safe(p, i, j, dim_x, dim_y, ctx);
    return (1 - omega) * (*p) + omega * p_gs;
}

static float pois_sor_fast(float *p, int i, int j, int dim_x, int dim_y,
                           void *ctx)
{
    struct pois_context *pois_ctx = (struct pois_context*)ctx;
    float omega = pois_ctx->omega;

    float p_sum = *(p - 1) + *(p + 1) + *(p - dim_x) + *(p + dim_x);
    float d_ij = pois_ctx->d[index(i, j, dim_x)];
    float p_gs = -0.25f * (pois_ctx->dx * d_ij - p_sum);

    return (1 - omega) * (*p) + omega * p_gs;
}

void poisson_solve(float *p, float *div, int dim_x, int dim_y, float dx,
                   int iters, float omega)
{
    for (int ij = 0; ij < dim_x * dim_y; ij++) {
        p[ij] = 0;
    }
    struct pois_context pois_ctx = { .d = div, .dx = dx, .omega = omega };
    for (int k = 0; k < iters; k++) {
        domain_iter_red_black(pois_sor_safe, pois_sor_fast, p, p, dim_x, dim_y,
                              &pois_ctx);
    }
}
