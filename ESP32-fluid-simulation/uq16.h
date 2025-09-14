#ifndef UQ16_H
#define UQ16_H

#include <cstdint>

#include "vector.h"

class UQ16 {
    public:
        uint16_t raw;

        UQ16() {}
        UQ16(float x) : raw(x+(0.5f)) {}

        // TODO: consider outlining operators explicitly?

        operator float() const { return float(this->raw); }
};

#endif
