// A very initial implementation of an integer that must be represented as a float, 
//  implemented with the volatile keyword to block optimizations. The result is forcing 
//  the compiler to use the l32i/s32i instructions thus allowing iram_float_t objects to 
//  be stored in IRAM, all while still using high level code
// This will probably be spun off into a separate library in the future.

#ifndef IRAM_FLOAT_H
#define IRAM_FLOAT_H

#include <cstdint>

#define BIT_REPR(x) (*reinterpret_cast<volatile uint32_t*>(&(x)))
#define BIT_INTERPR(x) (*reinterpret_cast<volatile float*>(&(x)))
#define BIT_REPR_CONST(x) (*reinterpret_cast<const volatile uint32_t*>(&(x)))
#define BIT_INTERPR_CONST(x) (*reinterpret_cast<const volatile float*>(&(x)))

class iram_float_t{
    public:
        iram_float_t(float value = 0) : _value(BIT_REPR(value)) {}

        iram_float_t& operator=(const iram_float_t &rhs){
            _value = rhs._value;
            return *this;
        }

        iram_float_t operator-() const{
            uint32_t a_raw = _value;
            float a = BIT_INTERPR(a_raw);
            return -a;
        }
        iram_float_t operator+(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            float a = BIT_INTERPR(a_raw), b = BIT_INTERPR(b_raw);
            return a+b;
        }
        iram_float_t operator-(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            float a = BIT_INTERPR(a_raw), b = BIT_INTERPR(b_raw);
            return a-b;
        }
        iram_float_t operator*(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            float a = BIT_INTERPR(a_raw), b = BIT_INTERPR(b_raw);
            return a*b;
        }
        iram_float_t operator/(const iram_float_t &rhs) const{
            uint32_t a_raw = _value, b_raw = rhs._value;
            float a = BIT_INTERPR(a_raw), b = BIT_INTERPR(b_raw);
            return a/b;
        }
        

        float as_float() const{
            float value = BIT_INTERPR_CONST(_value);
            return value;
        }
    private:
        volatile uint32_t _value;
};

inline iram_float_t operator+(const float &lhs, const iram_float_t &rhs){ return rhs+lhs; }
inline iram_float_t operator-(const float &lhs, const iram_float_t &rhs){ return rhs-lhs; }
inline iram_float_t operator*(const float &lhs, const iram_float_t &rhs){ return rhs*lhs; }
inline iram_float_t operator/(const float &lhs, const iram_float_t &rhs){ return rhs/lhs; }

#endif