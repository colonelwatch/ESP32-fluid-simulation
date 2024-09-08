#ifndef VECTOR_H
#define VECTOR_H

template<typename T>
class Vector2{
    public:
        T x;
        T y;
        
        Vector2& operator=(const Vector2 &rhs){
            this->x = rhs.x;
            this->y = rhs.y;
            return *this;
        }
        Vector2& operator+=(const Vector2 &rhs){
            this->x += rhs.x;
            this->y += rhs.y;
            return *this;
        }
        Vector2& operator-=(const Vector2 &rhs){
            this->x -= rhs.x;
            this->y -= rhs.y;
            return *this;
        }
        Vector2& operator*=(const float &rhs){
            this->x *= rhs;
            this->y *= rhs;
            return *this;
        }
        Vector2& operator/=(const float &rhs){
            this->x /= rhs;
            this->y /= rhs;
            return *this;
        }

        Vector2 operator-() const{ return {-this->x, -this->y}; }
        Vector2 operator+(const Vector2 &rhs) const{ return {this->x+rhs.x, this->y+rhs.y}; }
        Vector2 operator-(const Vector2 &rhs) const{ return {this->x-rhs.x, this->y-rhs.y}; }
        Vector2 operator*(const float &rhs) const{ return {this->x*rhs, this->y*rhs}; }
        Vector2 operator/(const float &rhs) const{ return {this->x/rhs, this->y/rhs}; }
};

// Scalar multiplication is commutative, so this fulfills that requirement
template<typename T>
inline Vector2<T> operator*(const float &lhs, const Vector2<T> &rhs){ return rhs*lhs; }

#endif