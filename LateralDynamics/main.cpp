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


int main(int argc, const char * argv[]) {
    Vehicle* car = new Vehicle();
    //Use Discrete Time Model to Observe Changes

    std::vector<std::vector<double>> vecToCsv;

    //Bicycle Kinematics
    while(t<10){
        //std::cout << "Starting Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        Dynamics::bicycleKinematics(car,dt);
        t += dt;
        HelperFunctions::storeAsVector(std::vector<double>{t,car->v,car->ψ},vecToCsv);
        //Dynamics::bicycleDynamics(car);
        std::cout << "Velocity of Car: " << car -> v << std::endl;
        std::cout << "Heading Angle, ѱ: " << car -> ψ/(2*M_PI) * 360<< " deg" << std::endl;
        std::cout << "--------" << std::endl;
    }
    std::vector<std::string> headers {"Time","Velocity","Heading angle"};
    HelperFunctions::toCsv("data1.csv",headers,vecToCsv);
    return 0;
}
