#ifndef VECTOR_H
#define VECTOR_H

#include <utility>
#include <initializer_list>

template<typename T> using TPromoted = decltype(std::declval<T>()*std::declval<float>());

template<typename T>
class Vector2{
    public:
        T x;
        T y;

        Vector2() {};
        template<typename U> Vector2(Vector2<U> v) : x((T)v.x), y((T)v.y) {}
        Vector2(std::initializer_list<T> l) {
            const T* ptr = l.begin();
            this->x = *(ptr++);
            this->y = *(ptr++);
        }
        
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
        Vector2 operator+(const Vector2 &rhs) const {
            return {this->x+rhs.x, this->y+rhs.y};
        }
        Vector2 operator-(const Vector2 &rhs) const { 
            return {this->x-rhs.x, this->y-rhs.y};
        }
        Vector2<TPromoted<T>> operator*(const float &rhs) const {
            return {this->x*rhs, this->y*rhs};
        }
        Vector2<TPromoted<T>> operator/(const float &rhs) const {
            return {this->x/rhs, this->y/rhs};
        }
};

// Scalar multiplication is commutative, so this fulfills that requirement
template<typename T>
inline Vector2<TPromoted<T>> operator*(const float &lhs, const Vector2<T> &rhs){ return rhs*lhs; }

template<typename T>
class Vector3 {
    public:
        T x;
        T y;
        T z;

        Vector3() {};
        template<typename U> Vector3(Vector3<U> v) : x((T)v.x), y((T)v.y), z((T)v.z) {}
        Vector3(std::initializer_list<T> l) {
            const T* ptr = l.begin();
            this->x = *(ptr++);
            this->y = *(ptr++);
            this->z = *(ptr++);
        }
        
        Vector3& operator=(const Vector3 &rhs){
            this->x = rhs.x;
            this->y = rhs.y;
            this->z = rhs.z;
            return *this;
        }
        Vector3& operator+=(const Vector3 &rhs){
            this->x += rhs.x;
            this->y += rhs.y;
            this->z += rhs.z;
            return *this;
        }
        Vector3& operator-=(const Vector3 &rhs){
            this->x -= rhs.x;
            this->y -= rhs.y;
            this->z -= rhs.z;
            return *this;
        }
        Vector3& operator*=(const float &rhs){
            this->x *= rhs;
            this->y *= rhs;
            this->z *= rhs;
            return *this;
        }
        Vector3& operator/=(const float &rhs){
            this->x /= rhs;
            this->y /= rhs;
            this->z /= rhs;
            return *this;
        }

        Vector3 operator-() const{ return {-this->x, -this->y, -this->z}; }
        Vector3 operator+(const Vector3 &rhs) const {
            return {this->x+rhs.x, this->y+rhs.y, this->z+rhs.z};
        }
        Vector3 operator-(const Vector3 &rhs) const{
            return {this->x-rhs.x, this->y-rhs.y, this->z-rhs.z};
        }
        Vector3<TPromoted<T>> operator*(const float &rhs) const{
            return {this->x*rhs, this->y*rhs, this->z*rhs};
        }
        Vector3<TPromoted<T>> operator/(const float &rhs) const{
            return {this->x/rhs, this->y/rhs, this->z/rhs};
        }
};

// Scalar multiplication is commutative, so this fulfills that requirement
template<typename T>
inline Vector3<TPromoted<T>> operator*(const float &lhs, const Vector3<T> &rhs){ return rhs*lhs; }

#endif