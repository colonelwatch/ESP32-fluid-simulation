#ifndef OPERATIONS_H
#define OPERATIONS_H

template<class T, class U>
using kernel_func_t = U (*)(T*, int i, int j, int dim_x, int dim_y, void* ctx);

static inline int index(int i, int j, int dim_x) {
    return dim_x*j+i;
}

template<class T, class U>
void domain_iter(kernel_func_t<T, U> expr_safe, kernel_func_t<T, U> expr_fast,
                 U *wrt, T *rd, int dim_x, int dim_y, void *ctx)
{
    int i_max = (dim_x)-1, j_max = (dim_y)-1;

    // Loop over the main body
    for (int j = 1; j < j_max; ++j) {
        for (int i = 1; i < i_max; ++i) {
            int ij = index(i, j, dim_x);
            wrt[ij] = expr_fast(&rd[ij], i, j, dim_x, dim_y, ctx);
        }
    }

    // Loop over the top and bottom boundaries (including corners)
    for (int i = 0; i <= i_max; ++i) {
        int ij_bottom = index(i, 0, dim_x), ij_top = index(i, j_max, dim_x);
        wrt[ij_bottom] = expr_safe(&rd[ij_bottom], i, 0, dim_x, dim_y, ctx);
        wrt[ij_top] = expr_safe(&rd[ij_top], i, j_max, dim_x, dim_y, ctx);
    }

    // Loop over the left and right boundaries
    for (int j = 1; j < j_max; ++j) {
        int ij_left = index(0, j, dim_x), ij_right = index(i_max, j, dim_x);
        wrt[ij_left] = expr_safe(&rd[ij_left], 0, j, dim_x, dim_y, ctx);
        wrt[ij_right] = expr_safe(&rd[ij_right], i_max, j, dim_x, dim_y, ctx);
    }
}

#endif
