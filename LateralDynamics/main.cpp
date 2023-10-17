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
const double dt = 0.1; //Time step
double t = 10; //10 seconds
//Functions are standardised, and used for seperate vehicle models
//Using as a header files is preferred

int main(int argc, const char * argv[]) {
    Vehicle* car = new Vehicle();
    //Use Discrete Time Model to Observe Changes
    
    //Number of passes
    
    for(int n =0;n < 100;++n){
        auto res = Dynamics::bicycle_kinematics(car -> lf, car -> lr, car -> v, car -> steeringRate, car -> steeringAngle);
        car -> v += (res[0] + res[1]) * dt;
        car -> steeringRate += res[2] * dt;
        car -> steeringAngle += res[3] * dt;
        t += dt;
        std::cout << "--------" << std::endl;
        std::cout << "Velocity of Car: " << car -> v << std::endl;
        std::cout << "Steering Rate, ѱ: " << car -> steeringRate << std::endl;
        std::cout << "Steering Angle, θ: " << car -> steeringAngle << std::endl;
    }
    
    
    return 0;
}
