#ifndef OPERATIONS_H
#define OPERATIONS_H

static inline int index(int i, int j, int dim_x) {
    return dim_x*j+i;
}

template<class T, class U>
void domain_iter(U (*expr_safe)(T*, int, int, int, int, void*),
        U (*expr_fast)(T*, int, int, int, int, void*), U *wrt,
        T *rd, int dim_x, int dim_y, void *ctx)
{
    int i_max = (dim_x)-1, j_max = (dim_y)-1;  // inclusive!
    int ij, ij_alt;

    // Loop over the main body
    for (int j = 1; j < j_max; ++j) {
        for (int i = 1; i < i_max; ++i) {
            ij = index(i, j, dim_x);
            wrt[ij] = expr_fast(&rd[ij], i, j, dim_x, dim_y, ctx);
        }
    }

    // Loop over the top and bottom boundaries (including corners)
    for (int i = 0; i <= i_max; ++i) {
        ij = index(i, 0, dim_x);
        wrt[ij] = expr_safe(&rd[ij], i, 0, dim_x, dim_y, ctx);
        ij_alt = index(i, j_max, dim_x);
        wrt[ij_alt] = expr_safe(&rd[ij_alt], i, j_max, dim_x, dim_y, ctx);
    }

    // Loop over the left and right boundaries
    for (int j = 1; j < j_max; ++j) {
        ij = index(0, j, dim_x);
        wrt[ij] = expr_safe(&rd[ij], 0, j, dim_x, dim_y, ctx);
        ij_alt = index(i_max, j, dim_x);
        wrt[ij_alt] = expr_safe(&rd[ij_alt], i_max, j, dim_x, dim_y, ctx);
    }
}

#endif
