#ifndef VECTOR_H
#define VECTOR_H

#include <iostream>

class Vector{
    public:
        float x;
        float y;
        
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

Vector& Vector::operator=(const Vector &rhs){
    this->x = rhs.x;
    this->y = rhs.y;
    return *this;
}

Vector& Vector::operator+=(const Vector &rhs){
    this->x += rhs.x;
    this->y += rhs.y;
    return *this;
}

Vector& Vector::operator-=(const Vector &rhs){
    this->x -= rhs.x;
    this->y -= rhs.y;
    return *this;
}

Vector& Vector::operator*=(const float &rhs){
    this->x *= rhs;
    this->y *= rhs;
    return *this;
}

Vector& Vector::operator/=(const float &rhs){
    this->x /= rhs;
    this->y /= rhs;
    return *this;
}

Vector Vector::operator-() const{
    Vector lhs = {-this->x, -this->y};
    return lhs;
}

Vector Vector::operator+(const Vector &rhs) const{
    Vector lhs = {this->x+rhs.x, this->y+rhs.y};
    return lhs;
}

Vector Vector::operator-(const Vector &rhs) const{
    Vector lhs = {this->x-rhs.x, this->y-rhs.y};
    return lhs;
}

Vector Vector::operator*(const float &rhs) const{
    Vector lhs = {this->x*rhs, this->y*rhs};
    return lhs;
}

// Flipped version of above for commutativity
inline Vector operator*(const float &lhs, const Vector &rhs){
    return rhs*lhs;
}

Vector Vector::operator/(const float &rhs) const{
    Vector lhs = {this->x/rhs, this->y/rhs};
    return lhs;
}

std::ostream& operator<<(std::ostream &os, const Vector &rhs)
{
    os << '(' << rhs.x << ',' << rhs.y << ')';
    return os;
}

#endif