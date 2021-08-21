// Not a very representative test, but it should be enough to verify a couple basic things
#include <cstdio>

#include "../ESP32-fluid-simulation/iram_float.h"

int main(){
    iram_float_t test0 = 1.0, test1 = 2.0, test2 = test0 + test1;
    
    printf("%f\n", test0.as_float());
    printf("%f\n", test1.as_float());
    printf("%f\n", test2.as_float());
    return 0;
}