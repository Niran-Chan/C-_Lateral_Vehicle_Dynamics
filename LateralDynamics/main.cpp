//
//  main.cpp
//  LateralDynamics
//
//  Created by Niranjan Gopinath 
//

#include <iostream>
#include <vector>
#include <string>

//Header Files
#include "Dynamics.hpp"
#include "Vehicle.hpp"
#include "HelperFunctions.hpp"

const double g = 9.81; //double instead of other data types to avoid casting
const double dt = 0.01; //Time step
double t = 0; //Starting Time Frame
//Functions are standardised, and used for seperate vehicle models
//Using as a header files is preferred

//Temporary Method to Store Input Data 
void storeAsVector(double var1,double var2,std::vector<double>& vec1,std::vector<double>& vec2)
{
    vec1.push_back(var1);
    vec2.push_back(var2);
}

int main(int argc, const char * argv[]) {
    Vehicle* car = new Vehicle();
    //Use Discrete Time Model to Observe Changes

    std::vector<double> time, vel;

    //Bicycle Kinematics
    while(t<10){
        //std::cout << "Starting Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        Dynamics::bicycleKinematics(car,dt);
        t += dt;
        storeAsVector(t,car->v,time,vel);
        //Dynamics::bicycleDynamics(car);
        std::cout << "Velocity of Car: " << car -> v << std::endl;
        std::cout << "Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        std::cout << "--------" << std::endl;
    }
    //std::vector<std::string> headers {"Time","Velocity"};
    //HelperFunctions::toCsv("data.csv",headers,time,vel);
    return 0;
}
