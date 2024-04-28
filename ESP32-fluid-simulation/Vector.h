#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>

template<typename T>
class Vector{
    public:
        T x;
        T y;
        
        Vector& operator=(const Vector &rhs){
            this->x = rhs.x;
            this->y = rhs.y;
            return *this;
        }
        Vector& operator+=(const Vector &rhs){
            this->x += rhs.x;
            this->y += rhs.y;
            return *this;
        }
        Vector& operator-=(const Vector &rhs){
            this->x -= rhs.x;
            this->y -= rhs.y;
            return *this;
        }
        Vector& operator*=(const float &rhs){
            this->x *= rhs;
            this->y *= rhs;
            return *this;
        }
        Vector& operator/=(const float &rhs){
            this->x /= rhs;
            this->y /= rhs;
            return *this;
        }

        Vector operator-() const{ return {-this->x, -this->y}; }
        Vector operator+(const Vector &rhs) const{ return {this->x+rhs.x, this->y+rhs.y}; }
        Vector operator-(const Vector &rhs) const{ return {this->x-rhs.x, this->y-rhs.y}; }
        Vector operator*(const float &rhs) const{ return {this->x*rhs, this->y*rhs}; }
        Vector operator/(const float &rhs) const{ return {this->x/rhs, this->y/rhs}; }
};

// Scalar multiplication is commutative, so this fulfills that requirement
template<typename T>
inline Vector<T> operator*(const float &lhs, const Vector<T> &rhs){ return rhs*lhs; }

#endif