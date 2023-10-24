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
const double dt = 0.01; //Time step
double t = 10; //10 seconds
//Functions are standardised, and used for seperate vehicle models
//Using as a header files is preferred

int main(int argc, const char * argv[]) {
    Vehicle* car = new Vehicle();
    //Use Discrete Time Model to Observe Changes
    
    //Bicycle Kinematics
    for(int n =0;n < 20;++n){
        Dynamics::bicycle_kinematics(car,dt);
        t += dt;
        std::cout << "Velocity of Car: " << car -> v << std::endl;
        std::cout << "Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        std::cout << "--------" << std::endl;
    }
    
    
    return 0;
}
