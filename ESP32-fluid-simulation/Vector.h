#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>

template<typename T>
class Vector{
    public:
        T x;
        T y;
        
        Vector& operator=(const Vector &rhs);
        Vector& operator+=(const Vector &rhs);
        Vector& operator-=(const Vector &rhs);
        Vector& operator*=(const float &rhs);
        Vector& operator/=(const float &rhs);

        Vector operator-() const;
        Vector operator+(const Vector &rhs) const;
        Vector operator-(const Vector &rhs) const;
        Vector operator*(const float &rhs) const;
        Vector operator/(const float &rhs) const;
};

template<typename T>
Vector<T>& Vector<T>::operator=(const Vector &rhs){
    this->x = rhs.x;
    this->y = rhs.y;
    return *this;
}

template<typename T>
Vector<T>& Vector<T>::operator+=(const Vector &rhs){
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

template<typename T>
Vector<T>& Vector<T>::operator-=(const Vector &rhs){
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

template<typename T>
Vector<T>& Vector<T>::operator*=(const float &rhs){
    this->x *= rhs;
    this->y *= rhs;
    return *this;
}

template<typename T>
Vector<T>& Vector<T>::operator/=(const float &rhs){
    this->x /= rhs;
    this->y /= rhs;
    return *this;
}

template<typename T>
Vector<T> Vector<T>::operator-() const{
    Vector lhs = {-this->x, -this->y};
    return lhs;
}

template<typename T>
Vector<T> Vector<T>::operator+(const Vector &rhs) const{
    Vector lhs = {this->x+rhs.x, this->y+rhs.y};
    return lhs;
}

template<typename T>
Vector<T> Vector<T>::operator-(const Vector &rhs) const{
    Vector lhs = {this->x-rhs.x, this->y-rhs.y};
    return lhs;
}

template<typename T>
Vector<T> Vector<T>::operator*(const float &rhs) const{
    Vector lhs = {this->x*rhs, this->y*rhs};
    return lhs;
}

// Flipped version of above for commutativity
template<typename T>
inline Vector<T> operator*(const float &lhs, const Vector<T> &rhs){
    return rhs*lhs;
}

template<typename T>
Vector<T> Vector<T>::operator/(const float &rhs) const{
    Vector lhs = {this->x/rhs, this->y/rhs};
    return lhs;
}

template<typename T>
std::ostream& operator<<(std::ostream &os, const Vector<T> &rhs)
{
    os << '(' << rhs.x << ',' << rhs.y << ')';
    return os;
}

#endif