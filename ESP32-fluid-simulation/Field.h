#ifndef FIELD_H
#define FIELD_H

#include <sstream>
#include <iomanip>

enum BoundaryCondition {DONTCARE, CLONE, NEGATIVE};

template<class T>
class Field{
    public:
        int N_i, N_j;
        BoundaryCondition bc;

        Field(int N_i, int N_j, BoundaryCondition bc);
        ~Field();

        // Boundary exists at i = -1, i = N_i, j = -1, j = N_j
        T& index(int i, int j);
        T index(int i, int j) const;
        void update_boundary(); // Make sure to call this after updating values!
        
        Field& operator=(const T *rhs);
        Field& operator=(const Field &rhs);

        std::string toString(int precision = -1, bool inside_only = true) const;
    private:
        T *_arr;
        int _inside_elems, _total_elems;
};

template<class T>
Field<T>::Field(int N_i, int N_j, BoundaryCondition bc){
    this->N_i = N_i;
    this->N_j = N_j;
    this->_inside_elems = N_i*N_j;
    this->_total_elems = (N_i+2)*(N_j+2);
    this->_arr = new T[_total_elems];
    this->bc = bc;
}

template<class T>
Field<T>::~Field(){
    delete[] this->_arr;
}

template<class T>
T& Field<T>::index(int i, int j){
    return this->_arr[1+(i+1)*(this->N_j+2)+j];
}

template<class T>
T Field<T>::index(int i, int j) const{
    return this->_arr[1+(i+1)*(this->N_j+2)+j];
}

template<class T>
void Field<T>::update_boundary(){
    if(this->bc == DONTCARE) return;
    else if(this->bc == CLONE){
        // corners
        this->index(-1, -1) = this->index(0, 0);
        this->index(N_i, -1) = this->index(N_i-1, 0);
        this->index(-1, N_j) = this->index(0, N_j-1);
        this->index(N_i, N_j) = this->index(N_i-1, N_j-1);
        
        // top and bottom sides
        for(int i = 0; i < N_i; i++){
            this->index(i, -1) = this->index(i, 0);
            this->index(i, N_j) = this->index(i, N_j-1);
        }
        for(int j = 0; j < N_j; j++){
            this->index(-1, j) = this->index(0, j);
            this->index(N_i, j) = this->index(N_i-1, j);
        }
    }
    else{ // this->bc == NEGATIVE
        // corners (negative of a negative!)
        this->index(-1, -1) = this->index(0, 0);
        this->index(N_i, -1) = this->index(N_i-1, 0);
        this->index(-1, N_j) = this->index(0, N_j-1);
        this->index(N_i, N_j) = this->index(N_i-1, N_j-1);
        
        // top and bottom sides
        for(int i = 0; i < N_i; i++){
            this->index(i, -1) = -this->index(i, 0);
            this->index(i, N_j) = -this->index(i, N_j-1);
        }
        for(int j = 0; j < N_j; j++){
            this->index(-1, j) = -this->index(0, j);
            this->index(N_i, j) = -this->index(N_i-1, j);
        }
    }
}

template<class T>
Field<T>& Field<T>::operator=(const T *rhs){
    for(int i = 0; i < this->N_i; i++)
        for(int j = 0; j < this->N_j; j++)
            this->index(i, j) = rhs[i*this->N_j+j];
    this->update_boundary();
    return *this;
}

template<class T>
Field<T>& Field<T>::operator=(const Field &rhs){
    for(int i = 0; i < this->N_i; i++)
        for(int j = 0; j < this->N_j; j++)
            this->index(i, j) = rhs.index(i, j);
    this->update_boundary();
    return *this;
}

template<class T>
std::string Field<T>::toString(int precision, bool inside_only) const{
    std::stringstream ss;
    if(precision != -1){
        ss << std::fixed << std::setprecision(precision);
    }
    if(inside_only){
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