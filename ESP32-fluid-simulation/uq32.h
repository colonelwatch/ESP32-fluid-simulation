#ifndef UQ32_H
#define UQ32_H

#include <cstdint>

#include "vector.h"

class UQ32 {
    public:
        uint32_t raw;

        UQ32() {}
        UQ32(float x) : raw(x + 0.5f) {}

        operator float() const { return float(this->raw); }
};

#endif
