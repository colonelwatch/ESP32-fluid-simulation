#ifndef FIELD_H
#define FIELD_H

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

        char* as_bytes(int *n_bytes, bool include_boundary = true);
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
char* Field<T>::as_bytes(int *n_bytes, bool include_boundary){
    int i_start, i_end, j_start, j_end;
    if(include_boundary){
        *n_bytes = this->_inside_elems*sizeof(T);
        i_start = 0;
        i_end = this->N_i;
        j_start = 0;
        j_end = this->N_j;
    }
    else{
        *n_bytes = this->_total_elems*sizeof(T);
        i_start = -1;
        i_end = this->N_i+1;
        j_start = -1;
        j_end = this->N_j+1;
    }

    char *bytes = new char[*n_bytes];
    T *elem_ptr = reinterpret_cast<T*>(bytes);

    for(int i = i_start; i < i_end; i++){
        for(int j = j_start; j < j_end; j++){
            (*elem_ptr++) = this->index(i, j);
        }
    }

    return bytes;
}

#endif