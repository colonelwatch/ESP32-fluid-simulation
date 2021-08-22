// Not a very representative test, but it should be enough to verify a couple basic things
#include <iostream>

#include "../ESP32-fluid-simulation/iram_float.h"

int main(){
    iram_float_t test0 = 1.0, test1 = 2.0, test2 = test0 + test1;
    
    std::cout << test0 << "\n" << test1 << "\n" << test2 << std::endl;
    return 0;
}