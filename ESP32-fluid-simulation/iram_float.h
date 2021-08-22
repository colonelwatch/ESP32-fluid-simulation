// A very initial implementation of an integer that must be represented as a float, 
//  implemented with the volatile keyword to block optimizations. The result is forcing 
//  the compiler to use the l32i/s32i instructions thus allowing iram_float_t objects to 
//  be stored in IRAM, all while still using high level code
// Original issue (and assembly solution): https://github.com/espressif/esp-idf/issues/3036
// This will probably be spun off into a separate library in the future.

#ifndef IRAM_FLOAT_H
#define IRAM_FLOAT_H

#include <cstdint>

#define BIT_REPR(x) (*reinterpret_cast<volatile uint32_t*>(&(x)))
#define BIT_INTERPR(x) (*reinterpret_cast<float*>(&(x)))

class iram_float_t{
    public:
        iram_float_t(float value = 0) : _value(BIT_REPR(value)) {}

        #ifdef ESP32
        void* operator new[] (size_t size){ // Forces allocation from IRAM
            return heap_caps_malloc(size, MALLOC_CAP_32BIT);
        }
        #endif

        operator float() const {
            uint32_t a_raw = _value;
            return BIT_INTERPR(a_raw);
        }
    private:
        volatile uint32_t _value;
};

#endif