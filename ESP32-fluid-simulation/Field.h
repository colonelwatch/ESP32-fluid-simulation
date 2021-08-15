#ifndef FIELD_H
#define FIELD_H

#include <sstream>
#include <iomanip>

enum BoundaryCondition {ZERO, CLONE, NEGATIVE}; // Maybe add zero and doesn't matter?

template<class T, BoundaryCondition bc>
class Field{
    public:
        int N_i, N_j;

        Field(int N_i, int N_j);
        ~Field();

        // Boundary exists at i = -1, i = N_i, j = -1, j = N_j
        T& index(int i, int j);
        T index(int i, int j) const;
        void update_boundary(); // Make sure to call this after updating values!
        
        Field& operator=(const T *rhs);
        Field& operator=(const Field &rhs);
        Field& operator+=(const Field &rhs);
        Field& operator-=(const Field &rhs);
        Field& operator*=(const float &rhs);
        Field& operator/=(const float &rhs);

        std::string toString(int precision = -1, bool nontrivial_only = true) const;
    private:
        T *_arr;
        int _nontrivial_elems, _total_elems;
};

template<class T, BoundaryCondition bc>
Field<T, bc>::Field(int N_i, int N_j){
    this->N_i = N_i;
    this->N_j = N_j;
    this->_nontrivial_elems = N_i*N_j;
    this->_total_elems = (N_i+2)*(N_j+2);
    this->_arr = new T[_total_elems];
}

template<class T, BoundaryCondition bc>
Field<T, bc>::~Field(){
    delete[] this->_arr;
}

template<class T, BoundaryCondition bc>
T& Field<T, bc>::index(int i, int j){
    return this->_arr[1+(i+1)*(N_j+2)+j];
}

template<class T, BoundaryCondition bc>
T Field<T, bc>::index(int i, int j) const{
    return this->_arr[1+(i+1)*(N_j+2)+j];
}

template<class T, BoundaryCondition bc>
void Field<T, bc>::update_boundary(){
    int factor;
    if(bc == ZERO) factor = 0;
    else if(bc == CLONE) factor = 1;
    else factor = -1; // this->bc == NEGATIVE

    this->index(-1, -1) = factor*this->index(0, 0);
    this->index(N_i, -1) = factor*this->index(N_i-1, 0);
    this->index(-1, N_j) = factor*this->index(0, N_j-1);
    this->index(N_i, N_j) = factor*this->index(N_i-1, N_j-1);
    for(int i = 0; i < N_i; i++){
        this->index(i, -1) = factor*this->index(i, 0);
        this->index(i, N_j) = factor*this->index(i, N_j-1);
    }
    for(int j = 0; j < N_j; j++){
        this->index(-1, j) = factor*this->index(0, j);
        this->index(N_i, j) = factor*this->index(N_i-1, j);
    }
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator=(const T *rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) = rhs[i*N_j+j];
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator=(const Field &rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) = rhs.index(i, j);
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator+=(const Field &rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) += rhs.index(i, j);
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator-=(const Field &rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) -= rhs.index(i, j);
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator*=(const float &rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) *= rhs;
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
Field<T, bc>& Field<T, bc>::operator/=(const float &rhs){
    for(int i = 0; i < N_i; i++)
        for(int j = 0; j < N_j; j++)
            this->index(i, j) /= rhs;
    this->update_boundary();
    return *this;
}

template<class T, BoundaryCondition bc>
std::string Field<T, bc>::toString(int precision, bool nontrivial_only) const{
    std::stringstream ss;
    if(precision != -1){
        ss << std::fixed << std::setprecision(precision);
    }
    if(nontrivial_only){
        for(int i = 0; i < N_i; i++){
            for(int j = 0; j < N_j; j++){
                ss << this->index(i, j);
                if(j != N_j-1) ss << " ";
            }
            if(i != N_i-1) ss << "\n";
        }
    }
    else{
        for(int i = -1; i < N_i+1; i++){
            for(int j = -1; j < N_j+1; j++){
                ss << this->index(i, j);
                if(j != N_j) ss << " ";
            }
            if(i != N_i) ss << "\n";
        }
    }
    return ss.str();
}

#endif