#ifndef Q15_H
#define Q15_H

#include <cstdint>

class q15_t{
    public:
        q15_t(int value = 0) : _value(value){}
        q15_t(double value){
            if(value > 0.99997) _value = 32767;
            else if(value < -1.0) _value = -32768;
            else _value = value*32768;
        }

        q15_t& operator=(const q15_t &rhs){
            _value = rhs._value;
            return *this;
        }
        q15_t& operator+=(const q15_t &rhs){
            _value += rhs._value;
            return *this;
        }
        q15_t& operator-=(const q15_t &rhs){
            _value -= rhs._value;
            return *this;
        }
        q15_t& operator*=(const q15_t &rhs){
            int32_t temp = (int32_t)_value*(int32_t)rhs._value;
            _value += (1 << 14);
            _value = temp >> 15;
            return *this;
        }
        q15_t& operator/=(const q15_t &rhs){
            int32_t temp = (int32_t)_value << 15;
            _value = temp/rhs._value;
            return *this;
        }

        q15_t operator-() const{ return -_value; }
        q15_t operator+(const q15_t &rhs) const{ return _value+rhs._value; }
        q15_t operator-(const q15_t &rhs) const{ return _value-rhs._value; }
        q15_t operator*(const q15_t &rhs) const{
            int32_t temp = (int32_t)_value*(int32_t)rhs._value;
            temp += (1 << 14);
            return temp >> 15;
        }
        q15_t operator/(const q15_t &rhs) const{
            int32_t temp = (int32_t)_value << 15;
            return temp/rhs._value;
        }

        int16_t as_int() const{ return _value; }
    private:
        int16_t _value;
};

inline q15_t operator*(const int &lhs, const q15_t &rhs){
    return rhs*lhs;
}

inline q15_t operator*(const float &lhs, const q15_t &rhs){
    return rhs*lhs;
}

#endif