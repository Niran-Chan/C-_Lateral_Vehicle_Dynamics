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
    for(int n =0;n < 100;++n){
        auto res = Dynamics::bicycle_kinematics(car -> lf, car -> lr, car -> v, car -> ψ ,car ->δf,car -> δr);
        //Output of function:
        double Vx = res[0],Vy = res[1];
        car -> v = sqrt(pow(Vx,2) + pow(Vy,2));
        car -> ψ = std::fmod((car -> ψ + res[2] * dt),(2*M_PI));
        t += dt;
        /*
        std::cout << "Velocity of Car: " << car -> v << std::endl;
        std::cout << "Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        std::cout << "--------" << std::endl;
         */
    }
    
    
    return 0;
}
