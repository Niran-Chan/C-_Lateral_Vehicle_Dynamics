//
//  main.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath 
//

#include <iostream>
#include <vector>
#include "Dynamics.hpp"
#include "Vehicle.hpp"

const double g = 9.81; //double instead of other data types to avoid casting
    
//Functions are standardised, and used for seperate vehicle models
//Using as a header files is preferred



int main(int argc, const char * argv[]) {
    Dynamics::test();
    Vehicle* car1 = new Vehicle();
    return 0;
}
