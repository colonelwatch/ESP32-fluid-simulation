// iram_float_t
// A float that must be stored in IRAM as an integer. The result is forcing the compiler 
//  to use the l32i/s32i instructions (using other instructions cause a LoadStoreError).
// Original issue (and assembly solution): https://github.com/espressif/esp-idf/issues/3036

#ifndef IRAM_FLOAT_H
#define IRAM_FLOAT_H

#include <cstdint>

class iram_float_t{
    public:
        iram_float_t(float value = 0) 
            : _value(*reinterpret_cast<volatile uint32_t*>(&value)) {}

        #ifdef ESP32
        void* operator new[] (size_t size){ // Allows allocation from IRAM
            return heap_caps_malloc(size, MALLOC_CAP_32BIT);
        }
        #endif

        operator float() const {
            uint32_t a_raw = _value;
            return *reinterpret_cast<float*>(&a_raw);
        }
    private:
        volatile uint32_t _value;
};

#endif