// A very initial implementation of an integer that must be represented as a float, 
//  implemented with the volatile keyword to block optimizations. The result is forcing 
//  the compiler to use the l32i/s32i instructions thus allowing iram_float_t objects to 
//  be stored in IRAM, all while still using high level code
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
        void operator delete[] (void* p){
            heap_caps_free(p);
        }
        #endif

        iram_float_t& operator=(const iram_float_t &rhs){
            _value = rhs._value;
            return *this;
        }

        iram_float_t operator-() const{
            uint32_t a_raw = _value;
            return -BIT_INTERPR(a_raw);
        }
        iram_float_t operator+(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            return BIT_INTERPR(a_raw)+BIT_INTERPR(b_raw);
        }
        iram_float_t operator-(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            return BIT_INTERPR(a_raw)-BIT_INTERPR(b_raw);
        }
        iram_float_t operator*(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            return BIT_INTERPR(a_raw)*BIT_INTERPR(b_raw);
        }
        iram_float_t operator/(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            return BIT_INTERPR(a_raw)/BIT_INTERPR(b_raw);
        }
        
        float as_float() const{
            uint32_t a_raw = _value;
            return BIT_INTERPR(a_raw);
        }
    private:
        volatile uint32_t _value;
};

inline iram_float_t operator+(const float &lhs, const iram_float_t &rhs){ return rhs+lhs; }
inline iram_float_t operator-(const float &lhs, const iram_float_t &rhs){ return rhs-lhs; }
inline iram_float_t operator*(const float &lhs, const iram_float_t &rhs){ return rhs*lhs; }
inline iram_float_t operator/(const float &lhs, const iram_float_t &rhs){ return rhs/lhs; }

#endif