#include "finitediff.h"

struct grad_sub_context {
    float *p;
    float two_dx_inv;
};

static float div_expr_safe(Vector2<float> *v, int i, int j, int dim_x, int dim_y,
        void *ctx)
{
    float two_dx_inv = *((float*)ctx);
    int i_max = dim_x-1, j_max = dim_y-1;

    // remember that ghost velocity is negative!
    float flow_sum = 0;
    flow_sum += (i > 0) ? -((v-1)->x) : v->x;
    flow_sum += (i < i_max) ? (v+1)->x : -(v->x);
    flow_sum += (j > 0) ? -((v-dim_x)->y) : v->y;
    flow_sum += (j < j_max) ? (v+dim_x)->y : -(v->y);

    return flow_sum * two_dx_inv;
}

static float div_expr_fast(Vector2<float> *v, int i, int j, int dim_x, int dim_y,
        void *ctx)
{
    float two_dx_inv = *((float*)ctx);
    float flow_sum = ((-(v-1)->x + (v+1)->x) + (-(v-dim_x)->y + (v+dim_x)->y));
    return flow_sum * two_dx_inv;
}

void divergence(float *div, Vector2<float> *v, int dim_x, int dim_y, float dx) {
    float two_dx_inv = 1.0f / (2.0f * dx);
    domain_iter(div_expr_safe, div_expr_fast, div, v, dim_x, dim_y,
        &two_dx_inv);
}

static Vector2<float> grad_sub_expr_safe(Vector2<float> *v, int i, int j,
        int dim_x, int dim_y, void *ctx)
{
    struct grad_sub_context *grad_ctx = (struct grad_sub_context*)ctx;
    float two_dx_inv = grad_ctx->two_dx_inv;
    int i_max = dim_x-1, j_max = dim_y-1;
    
    int ij = index(i, j, dim_x);
    float *p = &(grad_ctx->p[ij]);

    float p_left = (i > 0) ? *(p-1) : *p;
    float p_right = (i < i_max) ? *(p+1) : *p;
    float p_down = (j > 0) ? *(p-dim_x) : *p;
    float p_up = (j < j_max) ? *(p+dim_x) : *p;

    Vector2<float> out = {v->x - (p_right - p_left) * two_dx_inv,
        v->y - (p_up - p_down) * two_dx_inv};
    return out;
}

static Vector2<float> grad_sub_expr_fast(Vector2<float> *v, int i, int j,
        int dim_x, int dim_y, void *ctx)
{
    struct grad_sub_context *grad_ctx = (struct grad_sub_context*)ctx;
    float two_dx_inv = grad_ctx->two_dx_inv;
    
    int ij = index(i, j, dim_x);
    float *p = &grad_ctx->p[ij];

    Vector2<float> out = {v->x - (*(p+1) - *(p-1)) * two_dx_inv,
        v->y - (*(p+dim_x) - *(p-dim_x)) * two_dx_inv};
    return out;
}

void subtract_gradient(Vector2<float> *v, float *p, int dim_x, int dim_y,
        float dx)
{
    struct grad_sub_context pois_ctx = {
        .p = p,
        .two_dx_inv = 1.0f / (2.0f * dx)
    };
    domain_iter(grad_sub_expr_safe, grad_sub_expr_fast, v, v, dim_x, dim_y,
        &pois_ctx);
}